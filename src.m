%---------------------------------------%
% Main script for object-oriented MEng Refactor
% Sarah Swinton 
% 2243716S
% Initially Created: 08/10/21
clear all
%---------------------------------------%

%% Instantiation of Required Classes 
% rover{n} = typeOfRover(roverId, startPoint, targetPoint, desiredVelocity)
rover{1} = activeRover(1, [1, 1], [2, 2], 0.01);
% rover{2} = referenceRover(1, [1, 1], [2, 2], 0.01);
% 
% rover{3} = activeRover(2,{2,1},0.1);
% rover{4} = referenceRover(2,{2,1},0.1);
% 
% rover{5} = activeRover(3,{3,1},0.1); 
% rover{6} = referenceRover(3,{3,1},0.1); 
% 
% rover{7} = activeRover(4,{4,1},0.1);
% rover{8} = referenceRover(4,{4,1},0.1);
% 
% rover{9} = activeRover(5,{5,1},0.1); 
% rover{10} = referenceRover(5,{5,1},0.1); 