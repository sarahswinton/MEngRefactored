%-------------------------%
% OOP Refactor of MEng Project
% Sarah Swinton
% File Created 07/10/21
%-------------------------%
%% Rover Superclass Definition
classdef objRover
    % Attributes (changeable, visible to user)
    properties 
        roverId(1, 1) double {mustbeInteger, mustBePositive} 
        startPoint(2, 1) double 
        targetPoint(2, 1) double
        flags 
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
        
    end
    % Methods
end