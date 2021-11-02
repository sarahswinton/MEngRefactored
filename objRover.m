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
        
    end 
    
    % Attributes (unchangable, non-visible to user)
    properties (Constant, Hidden)
        natFrequency = 2.9;     
        safetyFactor = 2;
        obsSafeRadius = 0.2;
        rovSafetyFactor = 3.5 
        rovSafeRadius = 0.35;
        acceptanceRadius = 0.175;
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
            resultantVelocity = sqrt((obj.xo(1,1)^2 + (obj.xo(2,1)^2)));
            obj.xo(24,1) = resultantVelocity;
        end

        function evalCheckpoint(obj)
            %   Evaluate whether or not the current checkpoint should be
            %   incremented. 

        end 

        function obsAvoidance(obj)
            %   Evaluate whether obstacle avoidance is required. If so,
            %   adjust the rover's desired heading accordingly. 

        end 
        
        function obj = headingControl(obj, headingGains, h)
            %   Evaluate and update the heading of the rover object
            %   errorMappedPsi,ePsiIntegral,ePsiPrevious,stepSize
            psiKp = headingGains(1); 
            psiKi = headingGains(2);
            psiKd = headingGains(3); 
            ePsi = obj.errorMappedPsi;
            obj.ePsiIntegral = obj.ePsiIntegral + ePsi*h;
            ePsiDerivative = (ePsi - obj.ePsiPrevious)/h;
            obj.psiCS = (psiKp * ePsi)  + (psiKi * obj.ePsiIntegral) + (psiKd * ePsiDerivative);
            obj.ePsiPrevious = ePsi;
        end 

        function evalControlSignal(obj)
            %   Evaluate and update the heading of the rover object
            
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