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
            % Assign the current position of each rover
            obj.xPositions = xPos;
            obj.yPositions = yPos;
        end

        function crashStatus = collisionCheck(obj)
            crashStatus = zeros(obj.roverCount, 1);
            % Compare the positions of each rover
            for n = 1:1:obj.roverCount
                for m = n:1:obj.roverCount
                    if n ~= m 
                        distance = sqrt((obj.xPositions(n)-obj.xPositions(m))^2 +(obj.yPositions(n)-obj.yPositions(m))^2);
                        if distance <= 0.35
                            crashStatus(n) = 1;
                            crashStatus(m) = 1; 
                        end
                    end
                end
            end
        end


    end
end

