%% Active Rover Subclass
classdef activeRover < objRover
    %   Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % FDIR Properties 
        diagnosedFaultMode = 0; 
        faultMode = 0;
        faultInjectionTime = 0;
        faultState = 0;
        residualPosition = 0;
        faultDetected = 0; 
        faultDetectedPrev = 0;
        faultIsolated = 0;
        detectionTime= 0;  
    end
    
    methods
        function assignFault(obj,faultMode,injectionTime)
            % Assign fault type and time to activeRover instance
            obj.faultMode = faultMode;
            obj.faultInjectionTime = injectionTime; 
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

