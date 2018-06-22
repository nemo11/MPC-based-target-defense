
classdef UAV < UnderactuatedVehicle
%UAV Continuous time model of a UAV 
%
% v1 = UAV()
%
% State vector: x = [p;R(:,1);R(:,2);R(:,3)]
% 
% p - 3-d vector denoting the position of the vehicle.
% R - Rotation Matrix from body to inertial frame.
%
% StateEquation:
%
% pDot = R*[u(1);0;0];
% Rdot = R*S(u(2:4));
%
% See also Unicycle, Vehicle, UnderactuatedVehicle

% This file is part of VirtualArena.
%
% Copyright (C) 2012-14 Andrea Alessandretti
%
% andrea.alessandretti@{ist.utl.pt, epfl.ch}
% Automatic Control Laboratory, EPFL, Lausanne, Switzerland.
% Institute System and Robotics, IST, Lisbon, Portugal.
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.



    properties
       
    end
    
    methods
        
    function obj =  UAV(varargin)
            
             obj = obj@UnderactuatedVehicle(...
                'PositionSpaceDimension',3,...
                'Lvk',[1;0;0],'Lwk',eye(3),'nu',4,varargin{:});
    end
    
    end
    
    
end