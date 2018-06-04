classdef Single_Agent_RealVehicleROS < CtSystem

    % Model of a single agent system in turtlesim; 
    % In this model the turtle has a constant linear velocity provided by
    % the user. And the angular velocity is controlled with the help of the control variable.

    properties
        target
        location 
        velCmd
        velocity
    end
    
    methods
        
        function obj = Single_Agent_RealVehicleROS(location,...
                                                   velCmd,...
                                                   target,...
                                                   velocity)
                                               
          obj = obj@CtSystem('nx',3,'nu',1,'ny',3);
          obj.location = location;
          obj.target = target;
          obj.velCmd = velCmd;
          obj.velocity = velocity;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            % Publisher send u to the vehicle;
            velMsg = rosmessage(obj.velCmd);
            turtleData = receive(obj.location,10);
            turtle_pose = [turtleData.X;turtleData.Y;turtleData.Theta];
            distance = sqrt((turtle_pose(1)-obj.target(1))*(turtle_pose(1)-obj.target(1)) + (turtle_pose(2)-obj.target(2))*(turtle_pose(2)-obj.target(2)));    %distacne between target and turtle.
            if (distance >= 0.1)
                velMsg.Linear.X = obj.velocity;
                velMsg.Angular.Z = u(1);
            else
                velMsg.Linear.X = 0;
                velMsg.Angular.Z = 0;
            end    
            
            send(obj.velCmd,velMsg);
            xDot = [1*cos(x(3));1*sin(x(3));u(1)];          %state equation... eg. XDot = Ax + By.       
        end
        
        function y = h(obj,t,x,varargin)
             
            % Subscriber read position of the vehicle the vehicle;
            locationData = receive(obj.location , 10) ;
            y = [locationData.X;                           %state equation... eg. Y = Cx + Dy.
                 locationData.Y;
             	 locationData.Theta];         
        end            
        
    end 
    
end
