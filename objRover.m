%-------------------------%
% OOP Refactor of MEng Project
% Sarah Swinton
% File Created 07/10/21
%-------------------------%
%% Rover Superclass Definition
classdef objRover
    % Attributes (changeable, visible to user)
    properties 
        roverId(1, 1) double {mustBeInteger, mustBePositive} 
        startPoint(2, 1) double 
        targetPoint(2, 1) double
        xo = [0;0;0;0;0;0;startPoint(1);startPoint(2);0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
        xodot = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
        flags(3, 1) {mustBeInteger}
        desiredVelocity(1, 1) double {mustBeInteger, mustBePositive}
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
        function obj = objRover(roverId, startPoint, targetPoint, desiredVelocity)
            %   Construct an instance of the rover class
            obj.roverId = roverId;
            obj.startPoint = startPoint;
            obj.targetPoint = targetPoint;
            obj.desiredVelocity = desiredVelocity;
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
        
        function evalDesiredHeading(obj)
            %   Evaluate and update the heading of the rover object
            
        end 

        function evalControlSignal(obj)
            %   Evaluate and update the heading of the rover object
            
        end 

        function derivativeSegment(obj)
            %   Evaluate and update the heading of the rover object
                
        end 

        function integralSegment(obj, modelname, x, h, u)
            %   Integrate rover states using rk4int
            %   x is xo (i.e. the xcurr matrix)
            %   h is the step size
            %   u is the control signal
            k1 = h*feval(modelname, x, u);              % evaluate derivative k1
            k2 = h*feval(modelname, x+k1/2, u);         % evaluate derivative k2
            k3 = h*feval(modelname, x+k2/2, u);         % evaluate derivative k3
            k4 = h*feval(modelname, x+k3, u);		    % evaluate derivative k4
            obj.xo = x + (k1 + 2*k2 + 2*k3 + k4)/6;		% averaged output
        end 

    end
    % Methods
end