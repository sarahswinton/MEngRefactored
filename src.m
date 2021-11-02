%---------------------------------------%
% Main script for object-oriented MEng Refactor
% Sarah Swinton 
% 2243716S
% Initially Created: 08/10/21
clear
%---------------------------------------%
%% Instantiation of Required Classes 
% rover{n} = typeOfRover(roverId, startPoint, targetPoint, desiredVelocity)
rover{1} = activeRover(1, [1, 1], [1, 2], 0.01, "Four Wheel");
rover{2} = referenceRover(1, [1, 1], [1, 2], 0.01, "Four Wheel");

rover{3} = activeRover(2,[2,1],[2,2],0.01, "Four Wheel");
rover{4} = referenceRover(2,[2,1],[2,2],0.01, "Four Wheel");

rover{5} = activeRover(3,[3,1],[3,3],0.01, "Four Wheel"); 
rover{6} = referenceRover(3,[3,1],[3,3],0.01, "Four Wheel"); 

rover{7} = activeRover(4,[4,1],[4,4],0.01, "Four Wheel");
rover{8} = referenceRover(4,[4,1],[4,4],0.01, "Four Wheel");

rover{9} = activeRover(5,[5,1],[5,5],0.01, "Four Wheel"); 
rover{10} = referenceRover(5,[5,1],[5,5],0.01, "Four Wheel"); 

%% Simulation Initial Conditions
stepSize = 0.01;            
commsInterval = 0.01;       
endTime = 100;   

%% Online Path Following - Dynamic Segment
for time = 0:stepSize:endTime
    % Carry Out Simulation For Each Rover 
    for n = 1:1:length(rover)
        %----------------------------------%
        % Data Storage
        if rem(stepSize,commsInterval) == 0
        end
        %----------------------------------%
        

        %----------------------------------%
        % LOS Navigation

        % findVelocity(rover{n})
        % findWaypoint(rover{n})
        % checkForObstacles(rover{n})
        % findLOSAngle(rover{n})
        % avoidObstacles(rover{n})
        % mapPsi(rover{n})
        %----------------------------------%


        %----------------------------------%
        % Control Section

        % headingController(rover{n});
        % velocityController(rover{n});
        %----------------------------------%


        %----------------------------------%
        % Derivative Section 

        % rover{x}.xodot = roverModel(rover{n});
        %----------------------------------%


        %----------------------------------%
        % Integral Section
        % Temporary u value: 
        u = [0;0];
        rover{n}.xo = rk4int(rover{n},stepSize,u);
        %----------------------------------%

    end
end