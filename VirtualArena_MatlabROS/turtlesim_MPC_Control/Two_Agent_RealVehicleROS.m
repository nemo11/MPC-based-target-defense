classdef Two_Agent_RealVehicleROS   <   CtSystem

    properties
         
        flag = 0
        vm      % Attacker Velocity Magnitude
        vt      % Target Velocity Magnitude
        target_data_subscriber
        target_velocity_publisher
        attacker_data_subscriber
        attacker_velocity_publisher

    end
    
    methods
        
        function obj = Two_Agent_RealVehicleROS(target_data_subscriber,...
                                                target_velocity_publisher,...
                                                attacker_data_subscriber,...
                                                attacker_velocity_publisher,...
                                                vm,...
                                                vt)
            
            obj = obj@CtSystem('nx',7,'nu',1,'ny',7);            
            obj.target_data_subscriber = target_data_subscriber;
            obj.target_velocity_publisher = target_velocity_publisher;
            obj.attacker_data_subscriber = attacker_data_subscriber;            
            obj.attacker_velocity_publisher = attacker_velocity_publisher;
            obj.vm = vm;
            obj.vt = vt;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            % Publisher send u to the vehicle;
            target_velocity_Msg = rosmessage(obj.target_velocity_publisher);
            attacker_velocity_Msg = rosmessage(obj.attacker_velocity_publisher);
            
            target_Pose_Data = receive(obj.target_data_subscriber,10);
            attacker_Pose_data = receive(obj.attacker_data_subscriber,10);
            o1 = [target_Pose_Data.X;target_Pose_Data.Y];
            o2 = [attacker_Pose_data.X;attacker_Pose_data.Y];
            d = sqrt((o1(1)-o2(1))*(o1(1)-o2(1)) + (o1(2)-o2(2))*(o1(2)-o2(2)));    %distance between target and attacker.
            
            if obj.flag > 2             %to avoid initial random values.
                if (d >= 0.2)                    
                    target_velocity_Msg.Linear.X = obj.vt;
                    attacker_velocity_Msg.Linear.X = obj.vm;
                    target_velocity_Msg.Angular.Z = 0;
                    attacker_velocity_Msg.Angular.Z = double(subs(u(1)));
                else
                    target_velocity_Msg.Linear.X = 0;
                    attacker_velocity_Msg.Linear.X = 0;
                    target_velocity_Msg.Angular.Z = 0;
                    attacker_velocity_Msg.Angular.Z = 0;
                end    
                send(obj.attacker_velocity_publisher,attacker_velocity_Msg);
                send(obj.target_velocity_publisher,target_velocity_Msg);
            end   
            disp('--------publishing--------')
            
            %state equation ...... e.g xDot = Ax + Bu (for linear systems). 
            xDot = [obj.vm*cos(x(3));
                    obj.vm*sin(x(3));
                    u(1);
                    obj.vt*cos(0);
                    obj.vt*sin(0);
                    -obj.vt*cos(-x(7))+obj.vm*cos(x(3)-x(7));
                    (obj.vt*sin(-x(7))-obj.vm*sin(x(3)-x(7)))/x(6)];                
        end
        
        function y = h(obj,t,x,varargin)
        
            % Subscriber read position of the vehicle the vehicle;
            target_pose_data = receive(obj.target_data_subscriber,10);
            attacker_pose_data = receive(obj.attacker_data_subscriber,10);           
            theta = attacker_pose_data.Theta;
            
            % bounding theta of turtle between -pi to pi
            if( theta > 3.14 )
                theta = theta - 2*3.14;
            end
            if( theta < -3.14 )
                theta = theta + 2*3.14;
            end
            
            %state equation ...... e.g, Y = Cx + Du (for linear systems).  
            y = double([attacker_pose_data.X;
                 attacker_pose_data.Y;
                 theta;
                 target_pose_data.X;
                 target_pose_data.Y;
                 sqrt((target_pose_data.X - attacker_pose_data.X)^2 + (target_pose_data.Y - attacker_pose_data.Y)^2);
                 (atan2((target_pose_data.Y - attacker_pose_data.Y),(target_pose_data.X - attacker_pose_data.X)))]);
             
            disp('---taking output feedback---'); 
            obj.flag = obj.flag + 1;        
        end            
    end    
end