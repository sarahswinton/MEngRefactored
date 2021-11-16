classdef waypoints
    %WAYPOINTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x_waypoints = [];
        y_waypoints = [];
    end
    
    methods
        function obj = waypoints(xWP,yWP)
            % Construct an instance of the waypoint class
            obj.x_waypoints = xWP;
            obj.y_waypoints = yWP; 
        end
        
        function obj = objRover(roverId, startPoint, targetPoint, desiredVelocity, roverType)
            % Construct an instance of the rover class
            obj.roverId = roverId;
            obj.startPoint = startPoint;
            obj.targetPoint = targetPoint;
            obj.desiredVelocity = desiredVelocity;
            obj.xo(7) = startPoint(1);
            obj.xo(8) = startPoint(2);

            % Set Rover Type - this should be aapted so there are only 2
            % valid options.
            if roverType == "Four Wheel"
                obj.modelName = "fourWheelModel";
            elseif roverType == "Rocker Bogie"
                obj.modelName = "rockerBogieModel";
            end
        end

    end
end

