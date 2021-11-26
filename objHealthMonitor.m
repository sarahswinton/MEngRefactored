classdef objHealthMonitor < handle
    %HEALTHMONITOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        roverCount = 0;
        xPositions = [];
        yPositions = [];
        psiValues = [];
        detectionLog = [];
        faultLog = [];
    end
    
    methods
        function obj = objHealthMonitor(roverCount)
            %   Construct an instance of this class
            obj.roverCount = roverCount;
            obj.detectionLog = zeros(roverCount,1);
        end
        

        function assignRoverLocation(obj,xPos,yPos)
            % Assign the current position of each rover
            obj.xPositions = xPos;
            obj.yPositions = yPos;
        end


        function assignRoverYaw(obj,psi)
            obj.psiValues = psi;
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



        function faultTrue = faultDetector(obj,roverNo,residual,threshold)
            if residual >= threshold
                faultTrue = 1;
                obj.detectionLog(roverNo) = 1;
            else
                faultTrue = 0;
            end
        end

        function assignFault(obj,n,faultType)
            obj.faultLog(n) = faultType;
        end 
    end
end

