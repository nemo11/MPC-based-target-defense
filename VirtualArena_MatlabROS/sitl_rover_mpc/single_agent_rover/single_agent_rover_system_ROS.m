classdef single_agent_rover_system_ROS < CtSystem

    properties
         
        flag = 0
        count = 0
        LatLon_subscriber
        angle_subscriber
        velocity_publisher
        target
        velocity_magnitude
    end
    
    methods
        
        function obj = single_agent_rover_system_ROS(LatLon_subscriber, ...
                                                                 angle_subscriber, ...
                                                                 velocity_publisher, ...
                                                                 target, ...
                                                                 velocity_magnitude ...
                                                                 )
            
            obj = obj@CtSystem('nx',5,'nu',1,'ny',5);            
            obj.LatLon_subscriber = LatLon_subscriber;
            obj.angle_subscriber = angle_subscriber;
            obj.velocity_publisher = velocity_publisher;
            obj.target = target;
            obj.velocity_magnitude = velocity_magnitude;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            % Publisher send u to the vehicle;
            vel_Msg = rosmessage(obj.velocity_publisher);
            rover_LatLon = receive(obj.LatLon_subscriber,10);
            
            [utmX, utmY] = deg2utm(rover_LatLon.Latitude, rover_LatLon.Longitude);
            
            distance = sqrt((utmX-obj.target(1))^2 + (utmY-obj.target(2))^2);
            
            if obj.count > 2             %to avoid initial random values.
                disp('--------publishing--------');
%                 if (distance >= 2 && obj.flag == 1)
                if (distance >= 2)
                    vel_Msg.Linear.X = obj.velocity_magnitude;
                    vel_Msg.Angular.Z = u(1);
                    send(obj.velocity_publisher,vel_Msg);
                    obj.flag = 0;
                else
                    vel_Msg.Linear.X = 0;
                    vel_Msg.Angular.Z = 0;
                    send(obj.velocity_publisher,vel_Msg);
                end    
            end   
            
            %state equation ...... e.g xDot = Ax + Bu (for linear systems). 
            xDot = [obj.velocity_magnitude*cos(x(3));
                    obj.velocity_magnitude*sin(x(3));
                    u(1);
                    obj.velocity_magnitude*cos(x(3)-x(5));
                    -obj.velocity_magnitude*sin(x(3)-x(5))/x(4)];                
        end
        
        function y = h(obj,t,x,varargin)
        
            % Subscriber read position of the vehicle the vehicle;
            rover_LatLon = receive(obj.LatLon_subscriber,10);
            rover_angle = receive(obj.angle_subscriber,10);
            
            [utmX, utmY] = deg2utm(rover_LatLon.Latitude, rover_LatLon.Longitude);
            
            % bounding theta of turtle between -pi to pi
            rover_angle.Data = 90 - rover_angle.Data;
                        
            %state equation ...... e.g, Y = Cx + Du (for linear systems).  
            y = double([utmX;
                 utmY;
                 deg2rad(rover_angle.Data);
                 sqrt((obj.target(1) - utmX)^2 + (obj.target(2) - utmY)^2);
                 (atan2((obj.target(2) - utmY),(obj.target(1) - utmX)))]);
             
            disp('---taking output feedback---'); 
            obj.count = obj.count + 1;
            obj.flag = 1;
        end            
    end    
end