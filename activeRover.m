%% Active Rover Subclass
classdef activeRover < objRover
    %   Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % FDIR Properties 
        diagnosedFaultMode = 0; 
        faultMode = 0;
        faultInjectionTime = 0;
        residualPosition = 0;
        faultDetected = 0; 
        faultDetectedPrev = 0;
        faultIsolated = 0;
        detectionTime= 0;  
    end
    
    methods
        function obj = untitled(inputArg1,inputArg2)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

