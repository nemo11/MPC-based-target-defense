classdef two_agent_rover_system_ROS < CtSystem

    properties
         
        count = 0
        vm      % Attacker Velocity Magnitude
        vt      % Target Velocity Magnitude
        target_LatLon_subscriber
        target_angle_subscriber
        target_velocity_publisher
        attacker_LatLon_subscriber
        attacker_angle_subscriber
        attacker_velocity_publisher

        target_angle
        target_velocity_Msg
        attacker_velocity_Msg
    end
    
    methods
        
        function obj = two_agent_rover_system_ROS(target_LatLon_subscriber, ...
                                                              target_angle_subscriber, ...
                                                              target_velocity_publisher, ...
                                                              attacker_LatLon_subscriber, ...
                                                              attacker_angle_subscriber, ...
                                                              attacker_velocity_publisher, ...
                                                              vm, ...
                                                              vt ...
                                                              )
            
            obj = obj@CtSystem('nx',7,'nu',1,'ny',7);            
            obj.target_LatLon_subscriber = target_LatLon_subscriber;
            obj.target_angle_subscriber = target_angle_subscriber;
            obj.target_velocity_publisher = target_velocity_publisher;
            obj.attacker_LatLon_subscriber = attacker_LatLon_subscriber;            
            obj.attacker_angle_subscriber = attacker_angle_subscriber;            
            obj.attacker_velocity_publisher = attacker_velocity_publisher;
            obj.vm = vm;
            obj.vt = vt;
            
            obj.target_angle = receive(obj.target_angle_subscriber , 10);
            obj.target_velocity_Msg = rosmessage(obj.target_velocity_publisher);
            obj.attacker_velocity_Msg = rosmessage(obj.attacker_velocity_publisher);
            
          %  target_angle1 = 90 - obj.target_angle.Data;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            % Publisher send u to the vehicle;
            target_angle1 = 90 - obj.target_angle.Data;
            
            %state equation ...... e.g xDot = Ax + Bu (for linear systems). 
            xDot = [obj.vm*cos(x(3));
                    obj.vm*sin(x(3));
                    u(1);
                    obj.vt*cos(deg2rad(target_angle1));
                    obj.vt*sin(deg2rad(target_angle1));
                    -obj.vt*cos(deg2rad(target_angle1)-x(7))+obj.vm*cos(x(3)-x(7));
                    (obj.vt*sin(deg2rad(target_angle1)-x(7))-obj.vm*sin(x(3)-x(7)))/x(6)];                
        end
        
        function y = h(obj,t,x,varargin)
        
            % Subscriber read position of the vehicle the vehicle;
            target_LatLon = receive(obj.target_LatLon_subscriber,10);
            attacker_LatLon = receive(obj.attacker_LatLon_subscriber,10);           
            attacker_angle = receive(obj.attacker_angle_subscriber,10);
            
            [attacker_utmX,attacker_utmY]=deg2utm(attacker_LatLon.Latitude,attacker_LatLon.Longitude);
            [target_utmX,target_utmY]=deg2utm(target_LatLon.Latitude,target_LatLon.Longitude);

             %% bounding theta of rover between -pi to pi

            attacker_angle.Data = 90 - attacker_angle.Data;

%% ........
            %state equation ...... e.g, Y = Cx + Du (for linear systems).  
            y = [attacker_utmX;
                 attacker_utmY;
                 deg2rad(attacker_angle.Data);
                 target_utmX;
                 target_utmY;
                 sqrt((target_utmX - attacker_utmX)^2 + (target_utmY - attacker_utmY)^2);
                 (atan2((target_utmY - attacker_utmY),(target_utmX - attacker_utmX)))];
             
            disp('---taking output feedback---'); 
            obj.count = obj.count + 1;        
        end
        
        function pub(obj,t,x,u,varargin)

%             
%             target_LatLon = receive(obj.target_LatLon_subscriber , 10);
%             target_angle = receive(obj.target_angle_subscriber , 10);
%             attacker_LatLon = receive(obj.attacker_LatLon_subscriber , 10);
% %             attacker_angle = receive(obj.attacker_angle_subscriber , 10);
% 
%             %% bounding theta of rover between -pi to pi
% 
%             target_angle.Data = 90 - target_angle.Data;

 %% ......

            attacker_utmX = x(1);
            attacker_utmY = x(2); 
            target_utmX = x(4);
            target_utmY = x(5);

            distance = sqrt((target_utmX-attacker_utmX)^2 + (target_utmY-attacker_utmY)^2);    %distance between target and attacker.
            
            if obj.count > 2             %to avoid initial random values.
                disp('--------publishing--------');
                if (distance >= 0.002)                    
                    obj.target_velocity_Msg.Linear.X = obj.vt;
                    obj.attacker_velocity_Msg.Linear.X = obj.vm;
                    %obj.target_velocity_Msg.Angular.Z = 0;
                    obj.attacker_velocity_Msg.Angular.Z = u(1);
                    disp(u(1));
                else
                    obj.target_velocity_Msg.Linear.X = 0;
                    obj.attacker_velocity_Msg.Linear.X = 0;
                    obj.target_velocity_Msg.Angular.Z = 0;
                    obj.attacker_velocity_Msg.Angular.Z = 0;
                    disp(0);
                end    
                send(obj.attacker_velocity_publisher,obj.attacker_velocity_Msg);
                send(obj.target_velocity_publisher,obj.target_velocity_Msg);
            end   
        end
    end    
end