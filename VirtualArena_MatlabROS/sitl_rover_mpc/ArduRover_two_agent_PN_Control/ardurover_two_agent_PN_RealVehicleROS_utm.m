classdef ardurover_two_agent_PN_RealVehicleROS_utm < CtSystem

    properties
       vm
       vt
       attacker_LatLon_subscriber
       attacker_angle_subscriber
       attacker_velocity_publisher
       target_LatLon_subscriber
       target_angle_subscriber
       target_velocity_publisher
       ob_target_angle
       count = 0
    end
    
    methods
        
        function obj = ardurover_two_agent_PN_RealVehicleROS_utm(vm, ...
                                   vt, ...
                                   attacker_LatLon_subscriber, ...
                                   attacker_angle_subscriber, ...
                                   attacker_velocity_publisher, ...
                                   target_LatLon_subscriber, ...
                                   target_angle_subscriber, ...
                                   target_velocity_publisher ...
                                   )
            obj = obj@CtSystem('nx',5,'nu',1,'ny',5);
            obj.vm = vm;
            obj.vt = vt;
            obj.attacker_LatLon_subscriber = attacker_LatLon_subscriber;
            obj.attacker_angle_subscriber = attacker_angle_subscriber;
            obj.attacker_velocity_publisher = attacker_velocity_publisher;
            obj.target_LatLon_subscriber = target_LatLon_subscriber;
            obj.target_angle_subscriber = target_angle_subscriber;
            obj.target_velocity_publisher = target_velocity_publisher;
            obj.ob_target_angle = receive(obj.target_angle_subscriber,10);
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
             %% bounding theta of rover between -pi to pi

            target_angle1 = 90 - obj.ob_target_angle.Data;
            
            %% Publisher to send velocity to the vehicle;
            
            xDot = [obj.vm*cos(x(10));  %Velocity of Attacker in x-dircetion
                    obj.vm*sin(x(10));  %Velocity of Attacker in y-dircetion
                    obj.vt*cos(deg2rad(target_angle1));  %Velocity of Target in x-dircetion
                    obj.vt*sin(deg2rad(target_angle1));  %Velocity of Target in y-dircetion
                    obj.vt*cos(deg2rad(target_angle1)-x(9))-obj.vm*cos(x(10)-x(9));  %Velocity along Attacker-Target LOS
                    (obj.vt*sin(deg2rad(target_angle1)-x(9))-obj.vm*sin(x(10)-x(9)))/x(8);   %Angular velocity of above equation
                    u(1);
                    0];

        
        end
        
        function y = h(obj,t,x,varargin)
        
            %% Subscriber to read the position of the vehicle

            attacker_LatLon = receive(obj.attacker_LatLon_subscriber,10);
            attacker_angle = receive(obj.attacker_angle_subscriber,10);
            target_LatLon = receive(obj.target_LatLon_subscriber,10);
            target_angle = receive(obj.target_angle_subscriber,10);
           
            %% bounding theta of rover between -pi to pi

            attacker_angle.Data = 90 - attacker_angle.Data;
            target_angle.Data = 90 - target_angle.Data;

            %% Convert to UTM

            [attacker_utmX, attacker_utmY] = deg2utm(attacker_LatLon.Latitude, attacker_LatLon.Longitude);
            [target_utmX, target_utmY] = deg2utm(target_LatLon.Latitude, target_LatLon.Longitude);
            
            %% ......                               

            
            y = double([attacker_utmX;
                        attacker_utmY;
                        target_utmX;
                        target_utmY;
                        sqrt((target_utmX - attacker_utmX)^2 + (target_utmY - attacker_utmY)^2);
                        atan2(target_utmY - attacker_utmY, target_utmX - attacker_utmX);
                        deg2rad(attacker_angle.Data);
                        deg2rad(target_angle.Data)]);
        obj.ob_target_angle.Data = target_angle.Data;
        
        obj.count = obj.count + 1;
        end
        
        function pub(obj,t,x,u,varargin)
            
            attacker_vel_Msg = rosmessage(obj.attacker_velocity_publisher);
            target_vel_Msg = rosmessage(obj.target_velocity_publisher);
            
            attacker_LatLon = receive(obj.attacker_LatLon_subscriber,10);
%             attacker_angle = receive(obj.attacker_angle_subscriber,10);
            target_LatLon = receive(obj.target_LatLon_subscriber,10);
            
            %% bounding theta of rover between -pi to pi

             
            %% Convert to UTM

            [attacker_utmX, attacker_utmY] = deg2utm(attacker_LatLon.Latitude, attacker_LatLon.Longitude);
            [target_utmX, target_utmY] = deg2utm(target_LatLon.Latitude, target_LatLon.Longitude);
            
            %% ......                               

            attacker_target_distance = sqrt((attacker_utmX - target_utmX)^2 + (attacker_utmY - target_utmY)^2);
            
            if (obj.count > 2)
                if (attacker_target_distance >= 1)   
                    attacker_vel_Msg.Linear.X = obj.vm;
                    attacker_vel_Msg.Angular.Z = u(1);
                    target_vel_Msg.Linear.X = obj.vt;
                    target_vel_Msg.Angular.Z = 0.01;
                    disp(u(1));
                else
                    attacker_vel_Msg.Linear.X = 0;
                    attacker_vel_Msg.Angular.Z = 0;
                    target_vel_Msg.Linear.X = 0;
                    target_vel_Msg.Angular.Z = 0;
                end
            end
            
            send(obj.attacker_velocity_publisher, attacker_vel_Msg);
            send(obj.target_velocity_publisher, target_vel_Msg);
        
        end
    
    end
    
end