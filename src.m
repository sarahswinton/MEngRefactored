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
obsLocation(1,:) = 20;
obsLocation(2,:) = 14;

% Guidance
% Store rover ativity status and time at onset of inactivity
% roverInactive(n,1): 1 = Inactive, 0 = Active
% roverInactive(n,2): timestep at onset of activity 
roverInactive = zeros(length(rover),2);      

%% Environment Initialisation
    %%% Plotting Environment for paths 
    clf 
    hold on 
    % Define map variables 
xBoundary = 25;
yBoundary = 25;
% Define obstacles and terrain objects 
obsXOne = [0 0 5 5];
obsYOne = [3 25 25 7];
obsXTwo = [8 10 15 15];
obsYTwo = [0 18 20 0];
obsXThree = [5 5 25 25];
obsYThree = [24 25 25 24];
obsXFour =[24 24 25 25];   
obsYFour = [0 24 24 0];     
rockFieldX = [15 15 24 24]; 
rockFieldY = [10 15 18 13];
steepSlopeXOne = [0 0 6 6]; 
steepSlopeYOne = [2 24 24 6];
steepSlopeXTwo = [7 9 15.5 16 ]; 
steepSlopeYTwo = [0 19 21 0];
steepSlopeXThree = [6 6 25 25]; 
steepSlopeYThree = [23.5 25 25 23.5];
steepSlopeXFour = [23 23 25 25]; 
steepSlopeYFour = [0 23.5 23.5 0];
obstacleOneP = polyshape(obsXOne, obsYOne);
obstacleTwoP = polyshape(obsXTwo, obsYTwo);
obstacleThreeP = polyshape(obsXThree, obsYThree);
obstacleFourP = polyshape(obsXFour, obsYFour);
rockField = polyshape(rockFieldX,rockFieldY);
steepSlopeOne = polyshape(steepSlopeXOne, steepSlopeYOne);
steepSlopeTwo = polyshape(steepSlopeXTwo, steepSlopeYTwo);
steepSlopeThree = polyshape(steepSlopeXThree, steepSlopeYThree);
steepSlopeFour = polyshape(steepSlopeXFour, steepSlopeYFour);

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
for n = 1:1:width(rover)
    lastFullColumn = roverInactive(n,2); 
    stateData.n = stateOutput(:,(1:lastFullColumn),n);
end

% Plot 2D Martian Environment    
clf
figure(1)
axis([0 xBoundary 0 yBoundary])
xlabel('X Position (m)','fontsize',14)
ylabel('Y Position (m)','fontsize',14)
grid on 
hold on
% Plot environment objects
plot(steepSlopeOne, 'FaceColor', 'red', 'FaceAlpha', 0.2)
plot(steepSlopeTwo, 'FaceColor', 'red', 'FaceAlpha', 0.2)
plot(steepSlopeThree, 'FaceColor', 'red', 'FaceAlpha', 0.2)
plot(steepSlopeFour, 'FaceColor', 'red', 'FaceAlpha', 0.2)
plot(obstacleOneP, 'FaceColor', 'black', 'FaceAlpha', 0.8) 
plot(obstacleTwoP, 'FaceColor', 'black', 'FaceAlpha', 0.8)
plot(obstacleThreeP, 'FaceColor', 'black', 'FaceAlpha', 0.8)
plot(obstacleFourP, 'FaceColor', 'black', 'FaceAlpha', 0.8)
plot(rockField, 'FaceColor', 'green', 'FaceAlpha', 0.4)
% Plot Obstacles
th = 0:pi/50:2*pi;
for obsNo = 1:1:width(obsLocation)
    xObsRad.obsNo = rover{1}.obsSafeRadius * cos(th) + obsLocation(1,obsNo);
    yObsRad.obsNo = rover{1}.obsSafeRadius * sin(th) + obsLocation(2,obsNo);
    plot(xObsRad.obsNo,yObsRad.obsNo, 'b');
end
plot(obsLocation(1,:),obsLocation(2,:), 'o','MarkerSize',5, 'MarkerFaceColor',[0.75, 0, 0.75]);
% Plotting Path of the First Rover
plot(waypoints(1,:),waypoints(2,:), "ko");
hold on
plot(stateData.n(7,:,1),stateData.n(8,:,1), "r-")
xlabel("X Position (m)")
ylabel("Y Position (m)")
legend('','','','','','','','','','','', 'Waypoints','Measured Path')

