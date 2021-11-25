classdef objHealthMonitor
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
        
        function assignRoverLocation(obj,roverNo,xPos,yPos)
            obj.xPositions(roverNo) = xPos;
            obj.yPositions(roverNo) = yPos;
        end
    end
end

