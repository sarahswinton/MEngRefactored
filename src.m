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
% Store rover ativity status and time at onset of inactivity
% roverInactive(n,1): 1 = Inactive, 0 = Active
% roverInactive(n,2): timestep at onset of activity 
roverInactive = zeros(length(rover),2);       

%% Path Planning

% Generate Waypoints
%waypoints = [1.5,3,5,7;1.5,2,3,4];
waypoints = [2;1.5];

% Assign Waypoints To Rovers 
    % Requried functionality: assignment of different waypoints arrays
for roverNo = 1:1:length(rover)
    assignWaypoints(rover{roverNo}, waypoints);
end

%% Online Path Following - Dynamic Segment
for time = 0:stepSize:endTime
    % Carry Out Simulation For Each Rover 
    for n = 1:1:length(rover)
        % Only run the rover sim if the rover is active
        if roverInactive(n,1) == 0
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
    
            % Check if rover has reached its final waypoint
            if rover{n}.waypointCounter > width(waypoints)
                fprintf('Rover Number %i reached its final waypoint at time: %0.2f \n', n, time)
                roverInactive(n,1) = 1;
                roverInactive(n,2) = (time/stepSize)+1; 
                break       
            end
            
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
end

%% Terminal Segment 
% Data Prep: Remove zeros from the end of the stateOutput Array
% Note: last full row should be 9475
for n = 1:1:width(rover)
    lastFullColumn = roverInactive(n,2); 
    stateOutput_1 = stateOutput(:,(1:lastFullColumn),n);
end

% Plotting Path of the First Rover
clf
figure(1)
plot(waypoints(1,:),waypoints(2,:), "ko");
hold on
plot(stateOutput_1(7,:,1),stateOutput_1(8,:,1), "r-")
xlabel("X Position (m)")
ylabel("Y Position (m)")
legend("Desired Path","Measured Path")
xlim([0 10])
ylim([0 10])

