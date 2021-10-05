function [nodeCost] = PathPlannerCostClassification(pointCoord,maxStep,rockField,steepSlopeOne,steepSlopeTwo,steepSlopeThree,steepSlopeFour)
% This function looks up the heuristic cost of travelling to a new point 
% Terrain classes have the following cost: 
    % Steep Slope = 0.9; 
    % Rock Field = 0.5; 
    % Normal Path = 0.1;
% Cost values take into account the maxStep allowed for a move to a new
% node. This ensures that terrain cost will be reasonable in proportion to
% distance cost. 
%---------------------------------------------------------------------%

% Define point 
coordX = pointCoord(1);
coordY = pointCoord(2); 

% Define terrain costs 
nodeCost = 0;
ssCost = 0.9;
rfCost = 0.5; 
normalCost = 0.1; 

% Look up terrain cost of point
if isinterior(steepSlopeOne,coordX,coordY)
    nodeCost = maxStep * ssCost;
elseif isinterior(steepSlopeTwo,coordX,coordY)
    nodeCost = maxStep * ssCost; 
elseif isinterior(steepSlopeThree,coordX,coordY)
    nodeCost = maxStep * ssCost; 
elseif isinterior(steepSlopeFour,coordX,coordY)
    nodeCost = maxStep * ssCost;
elseif isinterior(rockField,coordX,coordY)
    nodeCost = maxStep * rfCost;
else 
    nodeCost = maxStep * normalCost;
end

end

