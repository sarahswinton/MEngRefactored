%---------------------------------------%
% Main script for object-oriented MEng Refactor
% Sarah Swinton 
% 2243716S
% Initially Created: 08/10/21
clear
clc
%---------------------------------------%

%------------%
% Bookmark: 
% Unfinished debugging of diagnosis
%------------% 
%% Instantiation of Required Classes 
% Rover Instantiation
% rover{n} = typeOfRover(roverId, startPoint, targetPoint, desiredVelocity, roverType)
rover{1} = activeRover(1, [1, 1], [16, 20], 0.1, "Four Wheel");
refRover{1} = referenceRover(1, [1, 1], [16, 20], 0.1, "Four Wheel");
rover{2} = activeRover(1, [2, 1], [18, 20], 0.1, "Four Wheel");
refRover{2} = activeRover(1, [2, 1], [18, 20], 0.1, "Four Wheel");
%rover{3} = activeRover(1, [3, 1], [20, 20], 0.1, "Four Wheel");
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
refRoverInactive = zeros(length(rover),2);   

% Path Planning
plannedArrivalTime = zeros(length(rover),1);
plannedPathLength = zeros(length(rover),1);
etaPlanned = zeros(length(rover),1);
crashTrue = 0;
safePath = 0;

% FDIR
reconfigMode = 2;
residualThreshold = 0.01;
faultTrue = zeros(width(rover),2);
detectionResiduals = zeros(width(rover),4);
diagnosisResiduals = zeros(width(rover),3);
reconfigurationRequest = zeros(width(rover),1);

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

% Prioritised Planning 
waypoints = [];
for roverNo = 1:1:length(rover)
    if roverNo ==1 
        [waypoints(1,:),waypoints(2,:)] = RRTStarOOP(rover{roverNo});
        assignWaypoints(rover{roverNo}, waypoints);
        assignWaypoints(refRover{roverNo}, waypoints);
        [plannedXOut(:,:),plannedPathLength(roverNo), etaPlanned(roverNo)] = childRoverFcnOOP(waypoints(1,:),waypoints(2,:), rover{roverNo});
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
            refRover{roverNo}.waypoints = [];
            crashTrue = 0; 
            % Plan New Path 
            [waypoints(1,:),waypoints(2,:)] = RRTStarOOP(rover{roverNo});
            assignWaypoints(rover{roverNo}, waypoints);
            assignWaypoints(refRover{roverNo}, waypoints);
            [plannedXOut(:,:),plannedPathLength(roverNo), etaPlanned(roverNo)] = childRoverFcnOOP(waypoints(1,:),waypoints(2,:), rover{roverNo});
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

        %----------------------------------%
        % Run active rover sim
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
            findVelocity(rover{n});
            range = distanceToWaypoint(rover{n});
            visibleObstacles = checkForObstacles(rover{n},obsNumber,obsLocation);
            distanceToObs = zeros(length(visibleObstacles),1);
            for j = 1:1:length(visibleObstacles)
                xDelta = visibleObstacles(1)-rover{n}.waypoints(1,rover{n}.waypointCounter);
                yDelta = visibleObstacles(2)-rover{n}.waypoints(2,rover{n}.waypointCounter);
                obsRange = sqrt((xDelta)^2+(yDelta)^2);
                distanceToObs(j) = obsRange;
            end
            waypointIncrementer(rover{n}, range, min(distanceToObs)); 
            if rover{n}.waypointCounter > width(rover{n}.waypoints)
                fprintf('Rover Number %i reached its final waypoint at time: %0.2f \n', n, time)
                roverInactive(n,1) = 1;
                roverInactive(n,2) = (time/stepSize)+1; 
            end
            if roverInactive(n,1) == 0
                LOSAngle = findLOSAngle(rover{n});
                LOSAngle = adjustForObstacles(rover{n}, visibleObstacles, LOSAngle);
                mapPsi(rover{n}, LOSAngle, stepSize);
                %----------------------------------%
        
        
                %----------------------------------%
                % Control Section
                headingControl(rover{n}, headingGains, stepSize);
                velocityControl(rover{n}, velocityGains, stepSize);
                u = [rover{n}.velCS;rover{n}.psiCS];
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
        %----------------------------------%


        %----------------------------------%
        % Run Reference Rover Sim
        if refRoverInactive(n,1) == 0
            %----------------------------------%
            % LOS Navigation
            findVelocity(refRover{n});
            range = distanceToWaypoint(refRover{n});
            visibleObstacles = checkForObstacles(refRover{n},obsNumber,obsLocation);
            distanceToObs = zeros(length(visibleObstacles),1);
            for j = 1:1:length(visibleObstacles)
                xDelta = visibleObstacles(1)-refRover{n}.waypoints(1,refRover{n}.waypointCounter);
                yDelta = visibleObstacles(2)-refRover{n}.waypoints(2,refRover{n}.waypointCounter);
                obsRange = sqrt((xDelta)^2+(yDelta)^2);
                distanceToObs(j) = obsRange;
            end
            waypointIncrementer(refRover{n}, range, min(distanceToObs)); 
            if refRover{n}.waypointCounter > width(refRover{n}.waypoints)
                fprintf('Ref Rover Number %i reached its final waypoint at time: %0.2f \n', n, time)
                refRoverInactive(n,1) = 1;
                refRoverInactive(n,2) = (time/stepSize)+1; 
            end
            %----------------------------------%
    
            if refRoverInactive(n,1) == 0
                %----------------------------------%
                LOSAngle = findLOSAngle(refRover{n});
                LOSAngle = adjustForObstacles(refRover{n}, visibleObstacles, LOSAngle);
                mapPsi(refRover{n}, LOSAngle, stepSize);
                %----------------------------------%
        
        
                %----------------------------------%
                % Control Section
                headingControl(refRover{n}, headingGains, stepSize);
                velocityControl(refRover{n}, velocityGains, stepSize);
                u = [refRover{n}.velCS;refRover{n}.psiCS];
                %----------------------------------%
        
        
                %----------------------------------%
                % Integral Section
                refRover{n}.xo = rk4int(refRover{n},stepSize,u);
                %----------------------------------%
            end
        end
    end
    %----------------------------------%

    %----------------------------------%
    % Health Monitoring Section
    
    % Assign rover locations for collision checking 
    xPos = zeros(width(rover),1);
    yPos = zeros(width(rover),1);
    psi = zeros(width(rover),1);
    for n = 1:1:width(rover)
        xPos(n) = rover{n}.xo(7);
        yPos(n) = rover{n}.xo(8);
        psi(n) = rover{n}.xo(12);
    end
    assignRoverLocation(healthMonitor,xPos,yPos)
    assignRoverYaw(healthMonitor,psi)
    
    % Check for rover collisions
    crashStatus = collisionCheck(healthMonitor);
    
    % Fault detection
    for n = 1:1:width(rover)
        if faultTrue(n,1) == 0
            positionResidual = sqrt((rover{n}.xo(7)-refRover{n}.xo(7))^2 + (rover{n}.xo(8)-refRover{n}.xo(8))^2);
            faultTrue(n) = faultDetector(healthMonitor,n,positionResidual,residualThreshold);
            if faultTrue(n,1) == 1
                faultTrue(n,2) = time;
                detectionResiduals(n,1) = positionResidual;
                detectionResiduals(n,2) = rover{n}.xo(12)-refRover{n}.xo(12);
                detectionResiduals(n,3) = rover{n}.xo(7);
                detectionResiduals(n,4) = rover{n}.xo(8);
                fprintf("Fault detected on rover %i at %0.2f s. \n", n, time)
            end 
        end
    end

    % Fault Isolation 
    for n = 1:1:width(rover)
        if faultTrue(n,1) == 1 && time == (faultTrue(n,2) + 1)
            fprintf("Start diagnosis \n")
            diagnosisResiduals(n,1) = sqrt((rover{n}.xo(7)-refRover{n}.xo(7))^2 + (rover{n}.xo(8)-refRover{n}.xo(8))^2);
            diagnosisResiduals(n,2) = rover{n}.xo(12)-refRover{n}.xo(12);
            diagnosisResiduals(n,3) = sqrt((rover{n}.xo(7)-detectionResiduals(n,3))^2+(rover{n}.xo(8)-detectionResiduals(n,4))^2);

            positionResidual = diagnosisResiduals(n,1) - detectionResiduals(n,1);
            yawResidual = diagnosisResiduals(n,2) - detectionResiduals(n,2);
            poseDeltaResidual = diagnosisResiduals(n,3);
            
            if positionResidual <= 0.01
                faultType = 2;
            elseif yawResidual <= 0.1
                faultType = 1;
                reconfigurationRequest(n) = 1;
            elseif abs(poseDeltaResidual) <= (0.25*rover{n}.desiredVelocity)
                faultType = 1; 
                reconfigurationRequest(n) = 1;
            else
                faultType = 4;
            end 
            assignFault(healthMonitor,n,faultType);
        end 
    end 

    % Target Reallocation  
    if reconfigMode == 1
        for n = 1:1:width(rover)
            if reconfigurationRequest(n) == 1
                % Find ETA of each rover
                eta = zeros(width(rover),1);
                for m = 1:1:width(rover)
                    if m == n 
                        eta(m) = 10000;
                    elseif roverInactive(n,1) == 0 
                        d = sqrt((rover{n}.targetPoint(1)-rover{m}.targetPoint(1))^2+(rover{n}.targetPoint(2)-rover{m}.targetPoint(2))^2);
                        v = rover{m}.desiredVelocity; 
                        t = d/v;
                        eta(m) = etaPlanned(m) + t;  
                    else 
                        eta(m) = 10000; 
                    end 
                end
                % Select best rover
                [minEta, minEtaIndex] = min(eta); 
                fprintf("The closest rover for allocation is %i. \n", minEtaIndex)
                % Add reallocated target 
                % update waypoints
                % assign waypoints
                updatedWP = [];
                updatedWP(1,:) = [rover{minEtaIndex}.waypoints(1,:) rover{n}.targetPoint(1)];
                updatedWP(2,:) = [rover{minEtaIndex}.waypoints(2,:) rover{n}.targetPoint(2)];
                assignWaypoints(rover{minEtaIndex},updatedWP);
                assignWaypoints(refRover{minEtaIndex},updatedWP);
                % Return reallocation
                reconfigurationRequest(n) = 0;
            end 
        end
    end 


    % Path Blending 
    if reconfigMode == 2
        distanceBetweenRovers = zeros(width(rover),1);
        newWaypointsX = [];
        newWaypointsY = [];
        for n = 1:1:width(rover)
            if reconfigurationRequest(n) == 1
                for j = 1:1:width(rover)
                    if n ~= j && (roverInactive(j,1) == 0)
                        distanceBetweenRovers(j) = sqrt((rover{n}.xo(7)-rover{j}.xo(7))^2 + (rover{n}.xo(8)-rover{j}.xo(8))^2);
                    else
                        distanceBetweenRovers(j) = 10000;       % Number large enough that it wont be selected 
                    end 
                end
    
                [minDistance, distanceIndex] = min(distanceBetweenRovers); 
                waypoints = rover{distanceIndex}.waypoints;
                waypointsCrashedRover = rover{n}.waypoints;
    
                % Compare WP Number for crashed rover and chosen rover
                waypointRemaining = zeros(2,1);
                waypointRemaining(1) = width(rover{n}.waypoints) - rover{n}.waypointCounter;
                waypointRemaining(2) = width(rover{distanceIndex}.waypoints) - rover{distanceIndex}.waypointCounter;
                % Select rover with fewer remaining waypoints
                [minWPRemain, waypointIndex] = min(waypointRemaining);
                if waypointIndex == 1
                    currentWPNumber = rover{n}.waypointCounter;
                else 
                    currentWPNumber = rover{distanceIndex}.waypointCounter;
                end 
                
                % Set max waypoints 
                if width(rover{n}.waypoints) >= width(rover{distanceIndex}.waypoints)
                    maxWaypoints = width(rover{distanceIndex}.waypoints);
                else 
                    maxWaypoints = width(rover{n}.waypoints);
                end 
    
                % Replace future WP with blended path WP
                for k = maxWaypoints:-1:currentWPNumber
                    midpointX = (rover{distanceIndex}.waypoints(1,k) + rover{n}.waypoints(1,k))/2;
                    midpointY = (rover{distanceIndex}.waypoints(2,k) + rover{n}.waypoints(2,k))/2;  
    %                 if k == maxWP 
    %                     targetSafe = 1; 
    %                     distanceBetweenTargets = zeros(roverNumber,1);
    %                     while targetSafe == 1
    %                         for checkTargets = 1:1:roverNumber
    %                             if checkTargets ~= n && checkTargets ~= j
    %                                 distanceBetweenTargets(checkTargets) = (sqrt((roverTargetsX(checkTargets)-midpointX)^2+(roverTargetsY(checkTargets)-midpointY)^2));
    %                             end 
    %                         end
    %                         if min(distanceBetweenTargets(distanceBetweenTargets>0)) < 1 
    %                             targetSafe = 1;
    %                             midpointY = midpointY - 0.1;
    %                         else
    %                             targetSafe = 0;
    %                         end 
    %                     end
    %                 end
                    newWaypointsX = [newWaypointsX midpointX];
                    newWaypointsY = [newWaypointsY midpointY];
                    
                end 

                newWaypointsX = [newWaypointsX rover{distanceIndex}.xo(7)];
                newWaypointsY = [newWaypointsY rover{distanceIndex}.xo(8)];


                % Add rover's previous waypoints to the list for correct
                % plotting output
                for wpNo = (rover{distanceIndex}.waypointCounter-1):-1:1
                    newWaypointsX = [newWaypointsX rover{distanceIndex}.waypoints(1,wpNo)];
                    newWaypointsY = [newWaypointsY rover{distanceIndex}.waypoints(2,wpNo)];
                end 
            
                pathBlendWaypoints = [flip(newWaypointsX);flip(newWaypointsY)];
                assignWaypoints(rover{distanceIndex},pathBlendWaypoints);
                assignWaypoints(refRover{distanceIndex},pathBlendWaypoints);
                % Return reallocation
                reconfigurationRequest(n) = 0;
    
            end 
        end 
    end 

    % Traffic Control 
    slow = zeros(width(rover),1); 
    for n = 1:1:width(rover)     
        if roverInactive(n,1) == 0 
            for j = n:1:width(rover)
                if n ~= j
                    distance = sqrt((rover{n}.xo(7)-rover{j}.xo(7))^2 + (rover{n}.xo(8)-rover{j}.xo(8))^2);
                    if distance <= 1.2  && distance > 0.8    % Action to be taken if rovers within 1.2 m of eachother)
                        slow(j) = slow(j) + 1;
                    elseif distance <= 0.8 && distance > 0.35
                        slow(j) = slow(j) + 2;
                    elseif distance <= 0.35 
                        % Mark rover n as crashed
                        roverInactive(n,1) = 1;
                        roverInactive(n,2) = (time/stepSize)+1; 
                        refRoverInactive(n,1) = 1;
                        refRoverInactive(n,2) = (time/stepSize)+1; 
                        
                        % Mark rover j as crashed
                        roverInactive(j,1) = 1;
                        roverInactive(j,2) = (time/stepSize)+1; 
                        refRoverInactive(j,1) = 1;
                        refRoverInactive(j,2) = (time/stepSize)+1; 
                    end
                end
            end 
        end 
    end
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