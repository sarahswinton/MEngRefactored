%---------------------------------------%
% Main script for object-oriented MEng Refactor
% Sarah Swinton 
% 2243716S
% Initially Created: 08/10/21
clear
%---------------------------------------%
%% Instantiation of Required Classes 
% rover{n} = typeOfRover(roverId, startPoint, targetPoint, desiredVelocity, roverType)
rover{1} = activeRover(1, [1, 1], [8, 6], 0.01, "Four Wheel");
% rover{2} = referenceRover(1, [1, 1], [1, 2], 0.01, "Four Wheel");
% 
% rover{3} = activeRover(2,[2,1],[2,2],0.01, "Four Wheel");
% rover{4} = referenceRover(2,[2,1],[2,2],0.01, "Four Wheel");
% 
% rover{5} = activeRover(3,[3,1],[3,3],0.01, "Four Wheel"); 
% rover{6} = referenceRover(3,[3,1],[3,3],0.01, "Four Wheel"); 
% 
% rover{7} = activeRover(4,[4,1],[4,4],0.01, "Four Wheel");
% rover{8} = referenceRover(4,[4,1],[4,4],0.01, "Four Wheel");
% 
% rover{9} = activeRover(5,[5,1],[5,5],0.01, "Four Wheel"); 
% rover{10} = referenceRover(5,[5,1],[5,5],0.01, "Four Wheel"); 

%% Simulation Initial Conditions
stepSize = 0.01;            
commsInterval = 0.01;       
endTime = 100;   
i = 0;

% Data Output 
timeSteps = endTime/stepSize;
stateOutput = zeros(24,timeSteps,length(rover));
timeOutput = zeros(timeSteps,length(rover));

% Control Gains: [Kp, Ki, Kd]
headingGains = [45; 3; 0.75];
velocityGains = [3; 13.75; 0]; 

% Obstacles
obsNumber = 1;
obsLocation(1,:) = 10;
obsLocation(2,:) = 10;

% Guidance

%% Path Planning

% Generate Waypoints
waypoints = [3,5,7;2,3,4];

% Assign Waypoints To Rovers 
assignWaypoints(rover{1}, waypoints);

%% Online Path Following - Dynamic Segment
for time = 0:stepSize:endTime
    % Carry Out Simulation For Each Rover 
    for n = 1:1:length(rover)
        %----------------------------------%
        % Data Storage
        if rem(stepSize,commsInterval) == 0
            % increment counter after each rover has been processed for the
            % current time step
            if n == 1
                i = i + 1;
            end
            % Store state 
            stateOutput(:,i,n) = rover{n}.xo;
            timeOutput(i,n) = time;
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
                  
        % Find if either visible obs or the rover are within acceptance radius 
        distanceToObs = zeros(length(visibleObstacles),1);
        for j = 1:1:length(visibleObstacles)
            xDelta = visibleObstacles(1)-rover{n}.waypoints(1,rover{n}.waypointCounter);
            yDelta = visibleObstacles(2)-rover{n}.waypoints(2,rover{n}.waypointCounter);
            obsRange = sqrt((xDelta)^2+(yDelta)^2);
            distanceToObs(j) = obsRange;
        end

        % Increment waypoint if necessary 
        waypointIncrementer(rover{n}, range, min(distanceToObs)); 
        
        % Find the rover's LOS Angle it's current waypoint
        LOSAngle = findLOSAngle(rover{n});
        
        % Adjust the rover's LOS Angle to enable obstacle avoidance
        LOSAngle = adjustForObstacles(rover{n}, visibleObstacles, LOSAngle);

        % Map the desired heading value
        mapPsi(rover{n}, LOSAngle, stepSize);
        %----------------------------------%


        %----------------------------------%
        % Control Section
        headingControl(rover{n}, headingGains, stepSize);
        velocityControl(rover{n}, velocityGains, stepSize);
        u = [rover{n}.velCS;rover{n}.psiCS];
        %----------------------------------%


        %----------------------------------%
        % Derivative Section 

        % rover{x}.xodot = roverModel(rover{n});
        %----------------------------------%


        %----------------------------------%
        % Integral Section
        rover{n}.xo = rk4int(rover{n},stepSize,u);
        %----------------------------------%

    end
end

%% Terminal Segment 
% Plotting Path of the First Rover
clf
figure(1)
plot(waypoints(1,:),waypoints(2,:), "ko");
hold on
plot(stateOutput(7,:,1),stateOutput(8,:,1), "r-")
xlabel("X Position (m)")
ylabel("Y Position (m)")
legend("Desired Path","Measured Path")
xlim([0 10])
ylim([0 10])

