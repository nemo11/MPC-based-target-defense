classdef RealVehicleROS_V4   <   CtSystem

    properties
        location1 
        velCmd1
        location2
        velCmd2
        flag = 0
    end
    
    methods
        
        function obj = RealVehicleROS_V4(location1,velCmd1,location2,velCmd2)
            
            obj = obj@CtSystem('nx',7,'nu',1,'ny',7);
            obj.location1 = location1;
            obj.location2 = location2;
            obj.velCmd1 = velCmd1;
            obj.velCmd2 = velCmd2;
            
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            % Publisher send u to the vehicle;
            
            velMsg1 = rosmessage(obj.velCmd1);
            velMsg2 = rosmessage(obj.velCmd2);
            
            turtle1Data = obj.location1.LatestMessage;
            turtle2Data = obj.location2.LatestMessage;
            
            o1 = [turtle1Data.X;turtle1Data.Y];
            o2 = [turtle2Data.X;turtle2Data.Y];
            
            d = sqrt((o1(1)-o2(1))*(o1(1)-o2(1)) + (o1(2)-o2(2))*(o1(2)-o2(2)));
            
            if obj.flag > 2             %to avoid initial random values
                
                if (d >= 0.2)

                    velMsg1.Linear.X = 0.2;
                    velMsg2.Linear.X = 0.5;


                    velMsg1.Angular.Z = 0;
                    disp (u(1));
                    velMsg2.Angular.Z = double(subs(u(1)));

                    else
                    velMsg1.Linear.X = 0;
                    velMsg2.Linear.X = 0;

                    velMsg1.Angular.Z = 0;
                    velMsg2.Angular.Z = 0;

                end    

                send(obj.velCmd2,velMsg2);
                send(obj.velCmd1,velMsg1);

            end   
            disp('--------subscribing--------')
            
            %state equation ...... e.g xDot = Ax + Bu (for linear systems). 
            xDot = [0.5*cos(x(3));
                    0.5*sin(x(3));
                    u(1);
                    0.2*cos(0);
                    0.2*sin(0);
                    -0.2*cos(-x(7))+0.5*cos(x(3)-x(7));
                    (0.2*sin(-x(7))-0.5*sin(x(3)-x(7)))/x(6)];
                
        end
        
        function y = h(obj,t,x,varargin)
        
            % Subscriber read position of the vehicle the vehicle;
            turtle1_Pose_Data = receive(obj.location1,10);
            turtle2_Pose_data = receive(obj.location2,10);
           
            theta = turtle2_Pose_data.Theta;
            
            % bounding theta of turtle between -pi to pi
            if( theta > 3.14 )
                theta = theta - 2*3.14;
            end
            if( theta < -3.14 )
                theta = theta + 2*3.14;
            end
            
            %state equation ...... e.g, Y = Cx + Du (for linear systems).  
            y = double([turtle2_Pose_data.X;
                 turtle2_Pose_data.Y;
                 theta;
                 turtle1_Pose_Data.X;
                 turtle1_Pose_Data.Y;
                 sqrt((turtle1_Pose_Data.X - turtle2_Pose_data.X)^2 + (turtle1_Pose_Data.Y - turtle2_Pose_data.Y)^2);
                 (atan2((turtle1_Pose_Data.Y - turtle2_Pose_data.Y),(turtle1_Pose_Data.X - turtle2_Pose_data.X)))]);
             
            disp('---taking output feedback---'); 
            obj.flag = obj.flag + 1;
        
       end
            
    end
        
end