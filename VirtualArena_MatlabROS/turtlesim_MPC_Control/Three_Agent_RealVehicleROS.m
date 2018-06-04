classdef Three_Agent_RealVehicleROS < CtSystem

    properties
        count = 0
        N
        K
        switchGuidanceLaw
        vm      % Attacker Velocity Magnitude
        vt      % Target Velocity Magnitude
        vd      % Defender Velocity Magnitude
        attacker_position_subscriber
        attacker_velocity_publisher
        target_position_subscriber
        target_velocity_publisher
        defender_position_subscriber
        defender_velocity_publisher
    end
    
    methods
        
        function obj = Three_Agent_RealVehicleROS(N, ...
                                          K, ...
                                          switchGuidanceLaw, ...
                                          vm, ...
                                          vt, ...
                                          vd, ...
                                          attacker_position_subscriber, ...
                                          attacker_velocity_publisher, ...
                                          target_position_subscriber, ...
                                          target_velocity_publisher, ...
                                          defender_position_subscriber, ...
                                          defender_velocity_publisher ...
                                          )
                                      
            obj = obj@CtSystem('nx',10,'nu',1,'ny',10);
            obj.N = N;
            obj.K = K;
            obj.switchGuidanceLaw = switchGuidanceLaw;
            obj.vm = vm;
            obj.vt = vt;
            obj.vd = vd;
            obj.attacker_position_subscriber = attacker_position_subscriber;
            obj.attacker_velocity_publisher = attacker_velocity_publisher;
            obj.target_position_subscriber = target_position_subscriber;
            obj.target_velocity_publisher = target_velocity_publisher;
            obj.defender_position_subscriber = defender_position_subscriber;
            obj.defender_velocity_publisher = defender_velocity_publisher;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            %% Publisher to send velocity to the vehicle;
            
            attacker_vel_Msg = rosmessage(obj.attacker_velocity_publisher);
            target_vel_Msg = rosmessage(obj.target_velocity_publisher);
            defender_vel_Msg = rosmessage(obj.defender_velocity_publisher);
            
            attacker_position = receive(obj.attacker_position_subscriber,10);
            target_position = receive(obj.target_position_subscriber,10);
            defender_position = receive(obj.defender_position_subscriber,10);
            
            attacker_target_distance = sqrt((attacker_position.X - target_position.X)^2 + (attacker_position.Y - target_position.Y)^2);
            defender_attacker_distance = sqrt((defender_position.X - attacker_position.X)^2 + (defender_position.Y - attacker_position.Y)^2);
            
            xDot = [obj.vm*cos(x(10));  %Velocity of Attacker in x-dircetion
                    obj.vm*sin(x(10));  %Velocity of Attacker in y-dircetion
                    obj.vd*cos(x(5));   %Velocity of Defender in x-dircetion
                    obj.vd*sin(x(5));   %Velocity of Defender in x-dircetion
                    u(1);               %Angular Velocity of Defender
                    obj.vt*cos(target_position.Theta);  %Velocity of Target in x-dircetion
                    obj.vt*sin(target_position.Theta);  %Velocity of Target in y-dircetion
                    obj.vt*cos(target_position.Theta-x(9))-obj.vm*cos(x(10)-x(8));  %Velocity along Attacker-Target LOS
                    (obj.vt*sin(target_position.Theta-x(9))-obj.vm*sin(x(10)-x(9)))/x(8);   %Angular velocity of above equation
%                     ((((-1).^floor(ceil(t/obj.switchGuidanceLaw)))+1)/2)*(obj.N*((obj.vt*sin(target_position.Theta-x(9))-obj.vm*sin(x(10)-x(9)))/x(8)))+(1-((((-1).^floor(ceil(t/obj.switchGuidanceLaw)))+1)/2))*(((-obj.K*(x(10)-x(9)))/obj.vm))];  %Angular Velocity of Target
                    obj.N*((obj.vt*sin(target_position.Theta-x(9))-obj.vm*sin(x(10)-x(9)))/x(8))];
                    
            if (obj.count > 2)
                if (attacker_target_distance >= 0.2 && defender_attacker_distance >= 0.2)   
                    attacker_vel_Msg.Linear.X = obj.vm;
                    attacker_vel_Msg.Angular.Z = xDot(10);
                    target_vel_Msg.Linear.X = obj.vt;
                    target_vel_Msg.Angular.Z = 0;
                    defender_vel_Msg.Linear.X = obj.vd;
                    defender_vel_Msg.Angular.Z = u(1);
                else
                    attacker_vel_Msg.Linear.X = 0;
                    attacker_vel_Msg.Angular.Z = 0;
                    target_vel_Msg.Linear.X = 0;
                    target_vel_Msg.Angular.Z = 0;
                    defender_vel_Msg.Linear.X = 0;
                    defender_vel_Msg.Angular.Z = 0;
                end
            end
            
            send(obj.attacker_velocity_publisher, attacker_vel_Msg);
            send(obj.target_velocity_publisher, target_vel_Msg);
            send(obj.defender_velocity_publisher, defender_vel_Msg);
        
        end
        
        function y = h(obj,t,x,varargin)
        
            %% Subscriber to read the position of the vehicle

            attacker_position = receive(obj.attacker_position_subscriber,10);
            target_position = receive(obj.target_position_subscriber,10);
            defender_position = receive(obj.defender_position_subscriber,10);
            
            if(defender_position.Theta > 3.14)
                defender_position.Theta = defender_position.Theta - 2^3.14;
            end

            if(defender_position.Theta < -3.14)
                defender_position.Theta = defender_position.Theta + 2^3.14;
            end
            
            y = double([attacker_position.X;
                        attacker_position.Y;
                        defender_position.X;
                        defender_position.Y;
                        defender_position.Theta;
                        target_position.X;
                        target_position.Y;
                        sqrt((target_position.X - attacker_position.X)^2 + (target_position.Y - attacker_position.Y)^2);
                        atan2(target_position.Y - attacker_position.Y, target_position.X - attacker_position.X);
                        attacker_position.Theta]);
        
             obj.count = obj.count + 1;
        end
    
    end
    
end
