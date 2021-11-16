classdef plannedPath
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        xLocation = 0;
        yLocation = 0;
    end
    
    methods
        function obj = plannedPath(xLocation,yLocation)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.xLocation = xLocation;
            obj.yLocation = yLocation;
        end
        
    end
end

