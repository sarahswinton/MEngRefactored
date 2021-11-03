%-------------------------%
% OOP Refactor of MEng Project
% Sarah Swinton
% File Created 07/10/21
%-------------------------%
%% Rover Superclass Definition
classdef objRover
    % Attributes (changeable, visible to user)
    properties 
        % User Defined Properties
        roverId = 0;             
        startPoint = [0 0];
        targetPoint = [0 0];
        xo = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
        xodot = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
        flags = [0 0 0];
        desiredVelocity = 0.01;
        
        % GNC Properties
        waypointCounter = 1;
        filteredPsi = 0;
        dFilteredPsi = 0;   
        state = 0;              
        psiLast= 0;    
        accumulate = 0;     
        accumulation = 0;
        eVelPrevious = 0; 
        eVelIntegral = 0;
        ePsiPrevious = 0;
        ePsiIntegral = 0;
        errorMappedPsi = 0;
        pathComplete = 0;  
        modelName = 0;
        psiCS = 0;
        velCS = 0;
        waypoints = [0,1;0,1];      % temporary value 
        
    end 
    
    % Attributes (unchangable, non-visible to user)
    properties (Constant, Hidden)
        natFrequency = 2.9;     
        safetyFactor = 2;
        obsSafeRadius = 0.2;
        rovSafetyFactor = 3.5 
        rovSafeRadius = 0.35;
    end

    % Methods (inherited by all rover sub-classes) 
    methods
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
        
        function findVelocity(obj)
            %   Evaluate and update the velocity of the rover object
            resultantVelocity = sqrt((obj.xo(1)^2 + (obj.xo(2)^2)));
            obj.xo(24,1) = resultantVelocity;
        end

        function distance = distanceToWaypoint(obj)
            %   Find the distance from the rover to it's next checkpoint.
            xDelta = obj.waypoints(1,obj.waypointCounter)-obj.xo(7);
            yDelta = obj.waypoints(2,obj.waypointCounter)-obj.xo(8);
            distance = sqrt((yDelta)^2+(xDelta)^2);
        end 

        function psiDesired = findDesiredHeading(obj)
            % Find the desire heading from the rover to its waypoint
            xDelta = obj.waypoints(1,obj.waypointCounter)-obj.xo(7);
            yDelta = obj.waypoints(2,obj.waypointCounter)-obj.xo(8);
            psiDesired = atan2(yDelta, xDelta);

        end

        function visibleObstacles = checkForObstacles(obj, obsNumber, obsLocation)
            % Set visbile obstacle to be empty
            visibleObstacles = [];

            % Check each obstacle to see if it's visible
            for i = 1:1:obsNumber
                % Find range of obstacle to Rover
                xDelta = obsLocation(1,i)-obj.xo(7);
                yDelta = obsLocation(2,i)-obj.xo(8);
                obsRange = sqrt((xDelta)^2+(yDelta)^2);

                % Find LOS angle between obstacle and rover
                psiObs =  atan2(yDelta,xDelta);
                visionAngle = abs(obj.xo(12)-psiObs);

                % Ensure [-pi,pi] boundary visionAngle issues are resolved
                if abs(obj.xo(12)) >= (0.95*pi)
                    visionAngle = abs(abs(psiNow)-abs(psiObs));
                end

                % Detect if objects are 'visible'
                if (obsRange <= 3)  && (visionAngle <= 1.0472) % i.e. only adjust heading if in danger of collision
                    visibleObstacles(end+1,:) = obsLocation(:,i);
                end

            end 
        end 

        function waypointIncrementer(obj, distanceToWaypoint, visibleObstacles)
            % Evaluate relevant acceptance radius
            if obj.modelName == "fourWheelModel"
                acceptanceRadius = 0.175;
            else
                acceptanceRadius = 2;   % Dummy value
            end 

            % Find if either visible obs or the rover are within this 
            % Bug: fix so this can handle multiple visible obstacles
            distanceToObs = zeros(length(visibleObstacles),1);
            for i = 1:1:length(visibleObstacles)
                xDelta = visibleObstacles(1)-obj.waypoints(1,obj.waypointCounter);
                yDelta = visibleObstacles(2)-obj.waypoints(2,obj.waypointCounter);
                obsRange = sqrt((xDelta)^2+(yDelta)^2);
                distanceToObs(i) = obsRange;
            end

            % if so, increment obj.waypointCounter
            if (distanceToWaypoint <= acceptanceRadius) 
                obj.waypointCounter = obj.waypointCounter + 1;
            elseif (min(distanceToObs) <= acceptanceRadius)
                obj.waypointCounter = obj.waypointCounter + 1;
            end 
        end
        
        function obj = headingControl(obj, headingGains, h)
            %   Evaluate and update the heading of the rover object
            %   errorMappedPsi,ePsiIntegral,ePsiPrevious,stepSize
            Kp = headingGains(1); 
            Ki = headingGains(2);
            Kd = headingGains(3); 
            e = obj.errorMappedPsi;
            obj.ePsiIntegral = obj.ePsiIntegral + e*h;
            eDerivative = (e - obj.ePsiPrevious)/h;
            obj.psiCS = (Kp * e)  + (Ki * obj.ePsiIntegral) + (Kd * eDerivative);
            obj.ePsiPrevious = e;
        end 

        function obj = velocityControl(obj, velocityGains, h)
            %   Evaluate and update the heading of the rover object
            %   errorMappedPsi,ePsiIntegral,ePsiPrevious,stepSize
            Kp = velocityGains(1); 
            Ki = velocityGains(2);
            Kd = velocityGains(3); 

            e = obj.desiredVelocity - obj.xo(24);
            obj.eVelIntegral = obj.eVelIntegral + e*h;
            eDerivative = (e - obj.eVelPrevious)/h;
            obj.velCS = (Kp * e)  + (Ki * obj.eVelIntegral) + (Kd * eDerivative);
            obj.eVelPrevious = e;
        end 

        function derivativeSegment(obj)
            %   Evaluate and update the heading of the rover object
                
        end 

        function xo = rk4int(obj, h, u)
            %   Integrate rover states using rk4int
            %   x is xo (i.e. the xcurr matrix)
            %   h is the step size
            %   u is the control signal
            k1 = h*feval(obj.modelName, obj.xo, u);              % evaluate derivative k1
            k2 = h*feval(obj.modelName, obj.xo+k1/2, u);         % evaluate derivative k2
            k3 = h*feval(obj.modelName, obj.xo+k2/2, u);         % evaluate derivative k3
            k4 = h*feval(obj.modelName, obj.xo+k3, u);		    % evaluate derivative k4
            xo = obj.xo + (k1 + 2*k2 + 2*k3 + k4)/6;		% averaged output
        end 

    end
    % Methods
end