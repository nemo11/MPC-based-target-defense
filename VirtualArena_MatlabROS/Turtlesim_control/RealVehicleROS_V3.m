classdef RealVehicleROS_V3 < CtSystem

    properties
        target
        location 
        velCmd
    end
    
    methods
        
        function obj = RealVehicleROS_V3(location,velCmd,target)
          obj = obj@CtSystem('nx',3,'nu',1,'ny',3);
        %  obj = obj@CtSystem('nx',2,'nu',1);
          obj.location = location;
          obj.target = target;
          obj.velCmd = velCmd;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            %% Publisher send u to the vehicle;
             
            
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

            xDot = [1*cos(x(3));1*sin(x(3));u(1)];

        
        end
        
        function y = h(obj,t,x,varargin)
        
            %% Subscriber read position of the vehicle the vehicle;

            locationData = obj.location.LatestMessage ;
            y = [locationData.X;
                 locationData.Y;
            	 locationData.Theta];
        
       end
        
    
    end
    
    
end
