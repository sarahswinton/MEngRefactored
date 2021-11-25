%---------------------------------------%
% Main script for object-oriented MEng Refactor
% Sarah Swinton 
% 2243716S
% Initially Created: 08/10/21
clear
clc
%---------------------------------------%
%% Instantiation of Required Classes 
% Rover Instantiation
% rover{n} = typeOfRover(roverId, startPoint, targetPoint, desiredVelocity, roverType)
rover{1} = activeRover(1, [1, 1], [16, 20], 0.1, "Four Wheel");
rover{2} = activeRover(1, [2, 1], [18, 20], 0.1, "Four Wheel");
rover{3} = activeRover(1, [3, 1], [20, 20], 0.1, "Four Wheel");
%rover{4} = activeRover(1, [4, 1], [22, 20], 0.1, "Four Wheel");
%rover{5} = activeRover(1, [5, 1], [22, 1], 0.1, "Four Wheel");

% Health Monitor Instantiation
% monitorName = objHealthMonitor(number of rovers);
healthMonitor = objHealthMonitor(width(rover));

%% Simulation Initial Conditions
stepSize = 0.01;            
commsInterval = 0.01;       
endTime = 400;   
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

% Path Planning
plannedArrivalTime = zeros(length(rover),1);
plannedPathLength = zeros(length(rover),1);
crashTrue = 0;
safePath = 0;

%% Environment Initialisation
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

% Generate Manual Waypoints
% waypoints(:,:,1) = [2;1.5];
% waypoints(:,:,2) = [3;1.5];
% waypoints(:,:,3) = [4;1.5];
% waypoints(:,:,4) = [5;1.5];
% waypoints(:,:,5) = [6;1.5];

% Prioritised Planning 
waypoints = [];
for roverNo = 1:1:length(rover)
    if roverNo ==1 
        [waypoints(1,:),waypoints(2,:)] = RRTStarOOP(rover{roverNo});
        assignWaypoints(rover{roverNo}, waypoints);
        [plannedXOut(:,:),plannedPathLength(roverNo), plannedArrivalTime(roverNo)] = childRoverFcnOOP(waypoints(1,:),waypoints(2,:), rover{roverNo});
        plannedPath{roverNo}.xLocation = plannedXOut(7,:);
        plannedPath{roverNo}.yLocation = plannedXOut(8,:);
        plannedXOut = [];
    else
        % Create new paths until a safe one is found
        pathCount = 1;
        safePath = 0;
        while safePath == 0
            % Empty previous unsafe path
            waypoints = [];
            plannedPath{roverNo}.xLocation = [];
            plannedPath{roverNo}.yLocation = [];
            plannedXOut = [];
            rover{roverNo}.waypoints = [];
            crashTrue = 0; 
            % Plan New Path 
            [waypoints(1,:),waypoints(2,:)] = RRTStarOOP(rover{roverNo});
            assignWaypoints(rover{roverNo}, waypoints);
            [plannedXOut(:,:),plannedPathLength(roverNo), plannedArrivalTime(roverNo)] = childRoverFcnOOP(waypoints(1,:),waypoints(2,:), rover{roverNo});
            plannedPath{roverNo}.xLocation = plannedXOut(7,:);
            plannedPath{roverNo}.yLocation = plannedXOut(8,:);
            plannedXOut = [];
            % Check new path against previous rover paths
            for j = 1:1:roverNo-1
                if width(plannedPath{roverNo}.xLocation) <= width(plannedPath{j}.xLocation)
                    for k = 1:1:width(plannedPath{j}.xLocation)
                        % Find the distance between both rovers 
                        if k <= width(plannedPath{roverNo}.xLocation)
                            distance = sqrt((plannedPath{roverNo}.xLocation(k)-plannedPath{j}.xLocation(k))^2 + (plannedPath{roverNo}.yLocation(k)-plannedPath{j}.yLocation(k))^2);
                        else
                            distance = sqrt((plannedPath{roverNo}.xLocation(end)-plannedPath{j}.xLocation(k))^2+(plannedPath{roverNo}.yLocation(end)-plannedPath{j}.yLocation(k))^2);
                        end 
                        % If collision detected, mark path as unsafe and
                        % break from loop 
                        if distance <= 0.35
                            crashTrue = 1;
                            safePath = 0;
                            break
                        end 
                    end 
                else
                    for k = 1:1:width(plannedPath{j}.xLocation)
                        % Find the distance between both rovers 
                        if k <= width(plannedPath{roverNo}.xLocation)
                            distance = sqrt((plannedPath{roverNo}.xLocation(k)-plannedPath{j}.xLocation(k))^2 + (plannedPath{roverNo}.yLocation(k)-plannedPath{j}.yLocation(k))^2);
                        else
                            distance = sqrt((plannedPath{roverNo}.xLocation(k)-plannedPath{j}.xLocation(end))^2+(plannedPath{roverNo}.yLocation(k)-plannedPath{j}.yLocation(end))^2);
                        end 
                        % If collision detected, mark path as unsafe and
                        % break from loop 
                        if distance <= 0.35
                            crashTrue = 1;
                            safePath = 0;
                            break
                        end 
                    end 
                end 
                % Mark path as safe or unsafe
                if crashTrue == 0 
                    safePath = 1;
                    fprintf("Safe path for rover %i after %i attempts. \r\n",roverNo,pathCount)
                    break
                else 
                    pathCount = pathCount + 1;
                    break
                end 
            end 
        end 
    end 
    waypoints = [];
end

%% Assign Faults 
assignFault(rover{1},1,100);

%% Online Path Following - Dynamic Segment
for time = 0:stepSize:endTime
    % Carry Out Simulation For Each Rover 
    for n = 1:1:length(rover)
        if n == 1
            i = i + 1;
        end
        % Only run the rover sim if the rover is active
        if roverInactive(n,1) == 0
            %----------------------------------%
            % Data Storage
            if rem(stepSize,commsInterval) == 0
                % increment counter after each rover has been processed for the
                % current time step
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
            if rover{n}.waypointCounter > width(rover{n}.waypoints)
                fprintf('Rover Number %i reached its final waypoint at time: %0.2f \n', n, time)
                roverInactive(n,1) = 1;
                roverInactive(n,2) = (time/stepSize)+1; 
                     
            end
            if roverInactive(n,1) == 0
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

                %----------------------------------%
                % FDIR Section
                if time == rover{n}.faultInjectionTime
                    rover{n}.faultState = rover{n}.faultMode;
                end
                %----------------------------------%
            end
        end
    end

    %----------------------------------%
    % Health Monitoring Section
    
    % Assign rover locations for collision checking 
    xPos = zeros(width(rover),1);
    yPos = zeros(width(rover),1);
    for n = 1:1:width(rover)
        xPos(n) = rover{n}.xo(7);
        yPos(n) = rover{n}.xo(8);
    end
    assignRoverLocation(healthMonitor,xPos,yPos)


    %----------------------------------%
end

%% Terminal Segment 

% Data Prep: Remove zeros from the end of the stateOutput Array
lastFullColumn = zeros(width(rover),1);
fixStep = 0;
for n = 1:1:width(rover)
    if roverInactive(n,2) == 0
        lastFullColumn(n) = timeSteps;
    else 
        lastFullColumn(n) = floor(roverInactive(n,2));
        % Check last full column is correct
        if stateOutput(7,(1:lastFullColumn(n)),n) == 0
            fixStep = 1;
        end 
        % if not, remove empty columns until it is 
        while fixStep == 1
            lastFullColumn(n) = lastFullColumn(n)-1;
            if stateOutput(7,(1:lastFullColumn(roverNo)),roverNo) ~= 0
                fixStep = 0;
            end 
        end 
    end 
end

% Plot rovers within 2D Martian Environment    
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
hold on 
% Plot Obstacles
th = 0:pi/50:2*pi;
for obsNo = 1:1:width(obsLocation)
    xObsRad.obsNo = rover{1}.obsSafeRadius * cos(th) + obsLocation(1,obsNo);
    yObsRad.obsNo = rover{1}.obsSafeRadius * sin(th) + obsLocation(2,obsNo);
    plot(xObsRad.obsNo,yObsRad.obsNo, 'b')
    hold on
end
plot(obsLocation(1,:),obsLocation(2,:), 'o','MarkerSize',5, 'MarkerFaceColor',[0.75, 0, 0.75])
hold on 
% Plot rover paths and waypoints
for roverNo = 1:1:width(rover)
    plot(rover{roverNo}.waypoints(1,:),rover{roverNo}.waypoints(2,:), "ko")
    hold on
    plot(stateOutput(7,(1:lastFullColumn(roverNo)),roverNo),stateOutput(8,(1:lastFullColumn(roverNo)),roverNo), "r-")
end
legend('','','','','','','','','','','', 'Waypoints','Measured Path')