classdef RealVehicleROS_V4 < CtSystem

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
            
            %% Publisher send u to the vehicle;
            
            velMsg1 = rosmessage(obj.velCmd1);
            velMsg2 = rosmessage(obj.velCmd2);
            
            turtle1Data = obj.location1.LatestMessage;
            turtle2Data = obj.location2.LatestMessage;
            
            o1 = [turtle1Data.X;turtle1Data.Y];
            o2 = [turtle2Data.X;turtle2Data.Y];
            
            d = sqrt((o1(1)-o2(1))*(o1(1)-o2(1)) + (o1(2)-o2(2))*(o1(2)-o2(2)));
            if obj.flag > 2
                %obj.flag = 0;
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
            disp('----------x--------')
            %disp(x);
            
            %disp (sqrt((x(1) - x(4))^2 + (x(2) - x(5))^2));
            xDot = [0.5*cos(x(3));
                    0.5*sin(x(3));
                    u(1);
                    0.2*cos(0);
                    0.2*sin(0);
                    -0.2*cos(-x(7))+0.5*cos(x(3)-x(7));
                    (0.2*sin(-x(7))-0.5*sin(x(3)-x(7)))/x(6)];
                
        end
        
        function y = h(obj,t,x,varargin)
        
            %% Subscriber read position of the vehicle the vehicle;
            D1 = receive(obj.location1,10);
            D2 = receive(obj.location2,10);
            %locationData = obj.location.LatestMessage ;
            theta = D2.Theta;
            if( theta > 3.14 )
                theta = theta - 2*3.14;
            end
            if( theta < -3.14 )
                theta = theta + 2*3.14;
            end
            
            y = double([D2.X;
                 D2.Y;
                 theta;
                 D1.X;
                 D1.Y;
                 sqrt((D1.X - D2.X)^2 + (D1.Y - D2.Y)^2);
                 (atan2((D1.Y - D2.Y),(D1.X - D2.X)))]);
             
            disp('--y-----'); 
            obj.flag = obj.flag + 1;
        
       end
            
    end
        
end