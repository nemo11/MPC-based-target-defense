classdef RealVehicleROS_V3 < CtSystem

    properties
        target
        location 
        velCmd
    end
    
    methods
        
        function obj = RealVehicleROS_V3(location,velCmd,target)
          %  obj = obj@CtSystem('nx',1,'nu',2,'ny',2);
        %  obj = obj@CtSystem('nx',2,'nu',1);
          obj.location = location;
          obj.target = target;
          obj.velCmd = velCmd;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            %% Publisher send u to the vehicle;
            %% <<< add here 
            
            velMsg = rosmessage(obj.velCmd);
            turtleData = obj.location.LatestMessage;
            o = [turtleData.X;turtleData.Y;turtleData.Theta];
            d = sqrt((o(1)-obj.target(1))*(o(1)-obj.target(1)) + (o(2)-obj.target(2))*(o(2)-obj.target(2)));
            if (d >= 0.1)
               
            velMsg.Linear.X = 1;
            velMsg.Angular.Z = u(1);
            send(obj.velCmd,velMsg);
           
            else
            velMsg.Linear.X = 0;
            velMsg.Angular.Z = 0;
            send(obj.velCmd,velMsg);
                
            end    
            w = [x , o ];
            disp('-matlab--turtle-')
            disp(w);
            disp('---');
            %xDot = 0;
            xDot = [1*cos(x(3));1*sin(x(3));u(1)];
            %xDot = [x(1);x(2);x(3)];
        
        end
        
        function y = h(obj,t,x,varargin)
        
            %% Subscriber read position of the vehicle the vehicle;
            %% <<< add here
            %% x and y of the uav     
           % locationData = receive(obj.location,10);
            locationData = obj.location.LatestMessage ;
            y = [locationData.X;
                 locationData.Y;
            	 locationData.Theta];
           % y = x(1:3);
            disp('--y--')
            disp(y);
            disp('--y--')
            
        
       end
        
    
    end
    
    
end
