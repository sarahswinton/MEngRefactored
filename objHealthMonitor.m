classdef objHealthMonitor < handle
    %HEALTHMONITOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        roverCount = 0;
        xPositions = [];
        yPositions = [];
    end
    
    methods
        function obj = objHealthMonitor(roverCount)
            %   Construct an instance of this class
            obj.roverCount = roverCount;
        end
        
        function assignRoverLocation(obj,xPos,yPos)
            obj.xPositions = xPos;
            obj.yPositions = yPos;
         end
    end
end

