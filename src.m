%---------------------------------------%
% Main script for object-oriented MEng Refactor
% Sarah Swinton 
% 2243716S
% Initially Created: 08/10/21
clear
%---------------------------------------%
%% Instantiation of Required Classes 
% rover{n} = typeOfRover(roverId, startPoint, targetPoint, desiredVelocity, roverType)
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
endTime = 10;   

% Control Gains: [Kp, Ki, Kd]
headingGains = [45; 3; 0.75];
velocityGains = [3; 13.75; 0]; 

% Obstacles
obsNumber = 1;
obsLocation(1,:) = [2,2];
obsLocation(2,:) = [1,5];

% Guidance




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

        % Find the rover's current velocity
        findVelocity(rover{n});
        
        % Find the distance between the rover and it's current waypoint
        range = distanceToWaypoint(rover{n});
        
        % Check for any visible obstacles
        visibleObstacles = checkForObstacles(rover{n},obsNumber,obsLocation);

        % Increment waypoint if necessary
        waypointIncrementer(rover{n}, range, visibleObstacles); 
        
        % Find the rover's LOS Angle it's current waypoint
        LOSAngle = findLOSAngle(rover{n});
        
        % Adjust the rover's LOS Angle to enable obstacle avoidance
        LOSAngle = adjustForObstacles(rover{n}, visibleObstacles, LOSAngle);

        % mapPsi(rover{n})
        %----------------------------------%


        %----------------------------------%
        % Control Section

        headingControl(rover{n}, headingGains, stepSize);
        velocityControl(rover{n}, velocityGains, stepSize);
        %----------------------------------%


        %----------------------------------%
        % Derivative Section 

        % rover{x}.xodot = roverModel(rover{n});
        %----------------------------------%


        %----------------------------------%
        % Integral Section
        % Temporary u value: 
        u = [rover{n}.psiCS;0];
        rover{n}.xo = rk4int(rover{n},stepSize,u);
        %----------------------------------%

    end
end