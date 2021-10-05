% 2243716 MEng Project 
% Fault Tolerant Coordination of Multiple Rovers for Planetary Exploration
%---------------------------------------------------------------------%
clc; clear all; clf;
%---------------------------------------------------------------------%
%%% Methodology
% Script for the testing of a rover health monitoring system. 
% Flag array values: 1 = true, 0 = false
% Flags: 
    % flagArray(1) = target reached 
    % flagArray(2) = obstacle avoidance engaged 
    % flagArray(3) = vehicle crash
% Fault Modes: 
    % faultMode = 0;    No faults
    % faultMode = 1;    Total loss of rover power
    % faultMode = 2;    LHS Actuator Partial Fault
    % faultMode = 3;    RHS Actuator Partial Fault
    % faultMode = 4;    Heading Sensor Fail 
%---------------------------------------------------------------------%
obsClear = []; 
roverNumber = 5;
obsSafeRadius = 0.2;
fullRunTime = tic;
fullRunTimeStart = tic;
planTimeStart = tic; 
totalTests = 1; 
testNo = zeros(totalTests,1);
faultInRover = zeros(totalTests,1);
faultInjectionTimeData = zeros(totalTests,1);
faultInjectionType = zeros(totalTests,1);
targetAllocatedTo = zeros(totalTests,1);
targetAcquiredBy = zeros(totalTests,roverNumber); 
trafficControl = zeros(totalTests,roverNumber); 
collisionData = zeros(totalTests,roverNumber); 
pathBlendedWith = zeros(totalTests,1);
%---------------------------------------------------------------------%


faultyRover = 0;
currentInjectionTime = 1;
reconfigMode = 2; 


%---------------------------------------------------------------------%
for testRun = 1:1:totalTests
    %%% Initial Segment
    % % Global values
    name = ["Alpha", "Bravo", "Charlie", "Delta", "Echo"];
    obs(1,:) = [16, 22, 20, 18, 21, 21, 20, 19, 17, 22];
    obs(2,:) = [12, 14, 12, 15, 12, 15, 14, 13, 14, 17];
    %etaArray = zeros(roverNumber,1);
    etaArray = [531.46,591.91,559.11,535.08,567.87];
    obsNumber = 10;
    stepSize = 0.01;            
    commsInterval = 0.02;       
    endTime = 800;   
    natFrequency = 2.9;     
    safetyFactor = 2;
    obsSafeRadius = 0.2;
    rovSafeRadius = 0.35;
    rovSafetyFactor = 3.5;
    obsPresent = 0; 
    acceptanceRadius = 0.175;
    reconfigurationArray = zeros(roverNumber,1);  
    crash = zeros(roverNumber,1); 
    pathComplete = zeros(roverNumber,1);
    targetReallocation = zeros(roverNumber,1); 
    pathEnd = zeros(roverNumber,1);
    waypointCounter = zeros(roverNumber,1) + 1;     waypointCounterRef = zeros(roverNumber,1) + 1;
    roverPathLengthRef = zeros(roverNumber,1);      roverPathLength = zeros(roverNumber,1);
    psiLastRef = zeros(roverNumber,1);              psiLast = zeros(roverNumber,1);
    stateRef = zeros(roverNumber,1);                state = zeros(roverNumber,1);
    accumulationRef = zeros(roverNumber,1);         accumulation = zeros(roverNumber,1);
    accumulateRef = zeros(roverNumber,1);           accumulate  = zeros(roverNumber,1);
    filteredPsiRef = zeros(roverNumber,1);          filteredPsi = zeros(roverNumber,1);
    dFilteredPsiRef = zeros(roverNumber,1);         dFilteredPsi = zeros(roverNumber,1);
    ePsiPreviousRef = zeros(roverNumber,1);         ePsiPrevious = zeros(roverNumber,1);
    eVelPreviousRef = zeros(roverNumber,1);         eVelPrevious = zeros(roverNumber,1);
    ePsiIntegralRef = zeros(roverNumber,1);         ePsiIntegral = zeros(roverNumber,1);
    eVelIntegralRef = zeros(roverNumber,1);         eVelIntegral = zeros(roverNumber,1);
    faultModeIsolated = zeros(roverNumber,1);
    targetAcquired = zeros(roverNumber,1);
    acquiredBy = zeros(roverNumber,1);
    slow = zeros(roverNumber,1);
    pathBlendSelection = 0; 
    counter = 1;
    counterRef = 1;
    residualThreshold = 0.01; 
    obsRovX = [];
    obsRovY = [];
    obsRov = [obsRovX; obsRovY];

    % FDIR Initialisation
    currentFaultMode = zeros(roverNumber,1);
    faultInjectionTime = zeros(roverNumber,1);
    residualPosition = zeros(roverNumber,1);
    residualPsi = zeros(roverNumber,1);
    faultDetected = zeros(roverNumber,1);
    faultDetectedPrevious = zeros(roverNumber,1);
    faultIsolated = zeros(roverNumber,1);
    detectionTime = zeros(roverNumber,1);
    xPositionDetection = zeros(roverNumber,1);
    yPositionDetection = zeros(roverNumber,1);
    xPositionIsolation = zeros(roverNumber,1);
    yPositionIsolation = zeros(roverNumber,1);
    roverMovementDiagnosis = zeros(roverNumber,1);
    enableTrafficControl = zeros(roverNumber,1);
    collisionType = zeros(roverNumber,1);
    rovObsDrawn = zeros(roverNumber,1);
    psiResidualDelta = 0; 
    positionResidualDelta = 0;
    allocatedTo = 0; 
    testNo(testRun) = testRun; 
    faultMode = zeros(roverNumber,1);  

    
    % Rover Alpha Initialisation 
    startXAlpha = 4;            startYAlpha = 1; 
    goalXAlpha = 22;            goalYAlpha = 2;
    xoARef = [0;0;0;0;0;0;startXAlpha;startYAlpha;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xdotARef = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xoA= [0;0;0;0;0;0;startXAlpha;startYAlpha;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xdotA = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    flagArrayARef = [];         flagArrayA = [];


    % Rover Bravo Initialisation 
    startXBravo = 3;            startYBravo = 1; 
    goalXBravo = 21;            goalYBravo = 3;
    xoBRef = [0;0;0;0;0;0;startXBravo;startYBravo;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xdotBRef = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xoB= [0;0;0;0;0;0;startXBravo;startYBravo;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xdotB = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    flagArrayBRef = [];         flagArrayB = [];

    % Rover Charlie Initialisation
    startXCharlie = 2;          startYCharlie = 1; 
    goalXCharlie = 20;          goalYCharlie = 4;
    xoCRef = [0;0;0;0;0;0;startXCharlie;startYCharlie;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xdotCRef = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xoC = [0;0;0;0;0;0;startXCharlie;startYCharlie;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xdotC = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    flagArrayCRef = [];         flagArrayC = [];

    % Rover Delta Initialisation 
    startXDelta = 1; startYDelta = 1; 
    goalXDelta = 18; goalYDelta = 5;
    xoDRef = [0;0;0;0;0;0;startXDelta;startYDelta;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xdotDRef = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xoD = [0;0;0;0;0;0;startXDelta;startYDelta;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xdotD = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    flagArrayDRef = [];         flagArrayD = [];

    % Rover Echo Initialisation
    startXEcho = 0; startYEcho = 1; 
    goalXEcho = 16; goalYEcho = 2;
    xoERef = [0;0;0;0;0;0;startXEcho;startYEcho;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xdotERef = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xoE = [0;0;0;0;0;0;startXEcho;startYEcho;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    xdotE = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
    flagArrayERef = [];         flagArrayE = [];
    
    
    xCurrentRef = [startXAlpha,startXBravo,startXCharlie,startXDelta,startXEcho];
    xCurrent = xCurrentRef;
    yCurrentRef = [startYAlpha,startYBravo,startYCharlie,startYDelta,startYEcho];
    yCurrent = yCurrentRef;
    roverTargetsX = [goalXAlpha; goalXBravo; goalXCharlie; goalXDelta; goalXEcho];
    roverTargetsY = [goalYAlpha; goalYBravo; goalYCharlie; goalYDelta; goalYEcho];
    roverStartX = [startXAlpha, startXBravo, startXCharlie, startXDelta, startXEcho];
    roverStartY = [startYAlpha, startYBravo, startYCharlie, startYDelta, startYEcho];

    % Select and apply rover fault and fault injection time 
    %faultInjectionTimeArray = [75,150,225,300,375];
    faultInjectionTimeArray = [75];
    if faultyRover == 5
        faultyRover = 1;
        currentInjectionTime = currentInjectionTime + 1; 
        if currentInjectionTime <= 5
            faultInjectionTime(faultyRover) = faultInjectionTimeArray(currentInjectionTime);
        else 
            currentInjectionTime = 1; 
            faultInjectionTime(faultyRover) = faultInjectionTimeArray(currentInjectionTime);
        end
    else 
        faultyRover = faultyRover + 1; 
    end 
    faultMode(faultyRover) = 0; 
    faultInjectionTime(faultyRover) = faultInjectionTimeArray(currentInjectionTime); 
    %---------------------------------------------------------------------%
    
    
    %---------------------------------------------------------------------%
    %%% Prioritised Planning 

    %%% RRT Path Plan Alpha 
    WPXAlpha = [];  WPYAlpha = [];  
    [WPXAlpha,WPYAlpha] = RRTStarFcn(startXAlpha, startYAlpha, goalXAlpha, goalYAlpha);
    pathWPXAlpha = flip(WPXAlpha);   % Start to goal WP coords in X
    pathWPYAlpha = flip(WPYAlpha);   % Start to goal WP coords in Y
    % Run path Alpha and return xOut at each comm interval
    [xOutAlpha,pathLengthA, etaAlpha] = ChildRoverFcn(pathWPXAlpha,pathWPYAlpha, startXAlpha, startYAlpha, obsClear);
    etaArray(1) = etaAlpha;

    %%% RRT Path Plan Bravo
    % Initialise Bravo Values 
    crashCount = 1; pathAttemptsB = 0;
    % Look for a safe rover path
    %while crashCount ~= 0
        WPXBravo = []; WPYBravo = [];
        [WPXBravo,WPYBravo] = RRTStarFcn(startXBravo, startYBravo, goalXBravo, goalYBravo);
        pathWPXBravo = flip(WPXBravo);    % Start to goal WP coords in X
        pathWPYBravo = flip(WPYBravo);    % Start to goal WP coords in Y 
        % Run path Bravo and return xOut each comm interval
        [xOutBravo,pathLengthB,etaBravo] = ChildRoverFcn(pathWPXBravo,pathWPYBravo, startXBravo, startYBravo, obsClear);
        % Correct elements in array
        [xCoordAlpha, yCoordAlpha, xCoordBravo,yCoordBravo] = PPStepCorrector(xOutAlpha(7,:),xOutAlpha(8,:),xOutBravo(7,:),xOutBravo(8,:));
        % Check each distance betwen rovers at each timestep 
        crashCount = 0;
        stepCount = size(xCoordAlpha,2);
        for timeStep = 1:1:stepCount
            pointDistance = sqrt((xCoordAlpha(timeStep)-xCoordBravo(timeStep))^2 +(yCoordAlpha(timeStep)-yCoordBravo(timeStep))^2);
            if pointDistance <= 0.35
                crashTrue = 1; 
                crashCount = 1;
                break
            end
        end
        pathAttemptsB = pathAttemptsB + 1;
    %end
    fprintf('Finding a safe path for Bravo took %i attempts. \n', pathAttemptsB)
    etaArray(2) = etaBravo;

    %%% RRT Path Plan Charlie
    % Initialise Charlie Values 
    crashCount = 1; pathAttemptsC = 0;
    % Look for a safe rover path
    %while crashCount ~= 0
        WPXCharlie = []; WPYCharlie = [];
        [WPXCharlie,WPYCharlie] = RRTStarFcn(startXCharlie, startYCharlie, goalXCharlie, goalYCharlie);
        pathWPXCharlie = flip(WPXCharlie);    % Start to goal WP coords in X
        pathWPYCharlie = flip(WPYCharlie);    % Start to goal WP coords in Y 
        % Run path Bravo and return xOut each comm interval
        [xOutCharlie,pathLengthC,etaCharlie] = ChildRoverFcn(pathWPXCharlie,pathWPYCharlie, startXCharlie, startYCharlie, obsClear);
        % Correct elements in array
        [xCoordAlpha, yCoordAlpha, xCoordCharlie,yCoordCharlie] = PPStepCorrector(xCoordAlpha,yCoordAlpha,xOutCharlie(7,:),xOutCharlie(8,:));
        [xCoordBravo, yCoordBravo, xCoordCharlie,yCoordCharlie] = PPStepCorrector(xCoordBravo,yCoordBravo,xCoordCharlie,yCoordCharlie);
        % Check each distance betwen rovers at each timestep 
        crashCount = 0;
        stepCount = size(xCoordAlpha,2);
        for timeStep = 1:1:stepCount
            pointDistanceToAlpha = sqrt((xCoordAlpha(timeStep)-xCoordCharlie(timeStep))^2 +(yCoordAlpha(timeStep)-yCoordCharlie(timeStep))^2);
            pointDistanceToBravo = sqrt((xCoordBravo(timeStep)-xCoordCharlie(timeStep))^2 +(yCoordBravo(timeStep)-yCoordCharlie(timeStep))^2);
            if pointDistanceToAlpha <= 0.35 || pointDistanceToBravo <= 0.35
                crashTrue = 1; 
                crashCount = 1;
                break
            end
        end
        pathAttemptsC = pathAttemptsC + 1;
    %end
    fprintf('Finding a safe path for Charlie took %i attempts. \n', pathAttemptsC)
    etaArray(3) = etaCharlie;

    %%% RRT Path Plan Delta
    % Initialise Delta Values 
    crashCount = 1; pathAttemptsD = 0;
    % Look for a safe rover path
    %while crashCount ~= 0
        WPXDelta = []; WPYDelta = [];
        [WPXDelta,WPYDelta] = RRTStarFcn(startXDelta, startYDelta, goalXDelta, goalYDelta);
        pathWPXDelta = flip(WPXDelta);    % Start to goal WP coords in X
        pathWPYDelta = flip(WPYDelta);    % Start to goal WP coords in Y 
        % Run path Bravo and return xOut each comm interval
        [xOutDelta,pathLengthD,crashTypeDelta,endTimeDelta] = ChildRoverFcn(pathWPXDelta,pathWPYDelta, startXDelta, startYDelta, obsClear);
        % Correct elements in array
        [xCoordAlpha, yCoordAlpha, xCoordDelta,yCoordDelta] = PPStepCorrector(xCoordAlpha,yCoordAlpha,xOutDelta(7,:),xOutDelta(8,:));
        [xCoordBravo, yCoordBravo, xCoordDelta,yCoordDelta] = PPStepCorrector(xCoordBravo,yCoordBravo,xCoordDelta,yCoordDelta);
        [xCoordCharlie, yCoordCharlie, xCoordDelta,yCoordDelta] = PPStepCorrector(xCoordCharlie,yCoordCharlie,xCoordDelta,yCoordDelta);
        % Check each distance betwen rovers at each timestep 
        crashCount = 0;
        stepCount = size(xCoordAlpha,2);
        for timeStep = 1:1:stepCount
            pointDistanceToAlpha = sqrt((xCoordAlpha(timeStep)-xCoordDelta(timeStep))^2 +(yCoordAlpha(timeStep)-yCoordDelta(timeStep))^2);
            pointDistanceToBravo = sqrt((xCoordBravo(timeStep)-xCoordDelta(timeStep))^2 +(yCoordBravo(timeStep)-yCoordDelta(timeStep))^2);
            pointDistanceToCharlie = sqrt((xCoordCharlie(timeStep)-xCoordDelta(timeStep))^2 +(yCoordCharlie(timeStep)-yCoordDelta(timeStep))^2);
            if pointDistanceToAlpha <= 0.35 || pointDistanceToBravo <= 0.35 || pointDistanceToCharlie <= 0.35
                crashTrue = 1; 
                crashCount = 1;
                break
            end
        end
        pathAttemptsD = pathAttemptsD + 1;
    %end
    fprintf('Finding a safe path for Delta took %i attempts. \n', pathAttemptsD)

    %%% RRT Path Plan Echo
    % Initialise Echo Values 
    crashCount = 1; pathAttemptsE = 0;
    % Look for a safe rover path
    %while crashCount ~= 0
        WPXEcho = []; WPYEcho = [];
        [WPXEcho,WPYEcho] = RRTStarFcn(startXEcho, startYEcho, goalXEcho, goalYEcho);
        pathWPXEcho = flip(WPXEcho);    % Start to goal WP coords in X
        pathWPYEcho = flip(WPYEcho);    % Start to goal WP coords in Y 
        % Run path Echo and return xOut each comm interval
        [xOutEcho,pathLengthE,crashTypeEcho,endTimeEcho] = ChildRoverFcn(pathWPXEcho,pathWPYEcho, startXEcho, startYEcho, obsClear);
        % Correct elements in array
        [xCoordAlpha, yCoordAlpha, xCoordEcho,yCoordEcho] = PPStepCorrector(xCoordAlpha,yCoordAlpha,xOutEcho(7,:),xOutEcho(8,:));
        [xCoordBravo, yCoordBravo, xCoordEcho,yCoordEcho] = PPStepCorrector(xCoordBravo,yCoordBravo,xCoordEcho,yCoordEcho);
        [xCoordCharlie, yCoordCharlie, xCoordEcho,yCoordEcho] = PPStepCorrector(xCoordCharlie,yCoordCharlie,xCoordEcho,yCoordEcho);
        [xCoordDelta, yCoordDelta, xCoordEcho,yCoordEcho] = PPStepCorrector(xCoordDelta,yCoordDelta,xCoordEcho,yCoordEcho);
        % Check each distance betwen rovers at each timestep 
        crashCount = 0;
        stepCount = size(xCoordAlpha,2);
        for timeStep = 1:1:stepCount
            pointDistanceToAlpha = sqrt((xCoordAlpha(timeStep)-xCoordEcho(timeStep))^2 +(yCoordAlpha(timeStep)-yCoordEcho(timeStep))^2);
            pointDistanceToBravo = sqrt((xCoordBravo(timeStep)-xCoordEcho(timeStep))^2 +(yCoordBravo(timeStep)-yCoordEcho(timeStep))^2);
            pointDistanceToCharlie = sqrt((xCoordCharlie(timeStep)-xCoordEcho(timeStep))^2 +(yCoordCharlie(timeStep)-yCoordEcho(timeStep))^2);
            pointDistanceToDelta = sqrt((xCoordDelta(timeStep)-xCoordEcho(timeStep))^2 +(yCoordDelta(timeStep)-yCoordEcho(timeStep))^2);
            if pointDistanceToAlpha <= 0.35 || pointDistanceToBravo <= 0.35 || pointDistanceToCharlie <= 0.35 || pointDistanceToDelta <= 0.35
                crashTrue = 1; 
                crashCount = 1;
                break
            end
        end
        pathAttemptsE = pathAttemptsE + 1;
    %end
    fprintf('Finding a safe path for Echo took %i attempts. \n', pathAttemptsE)
    %---------------------------------------------------------------------%
    

    %---------------------------------------------------------------------%
    originalWP = zeros(roverNumber,1);
    originalWP(1) = size(pathWPXAlpha,2);
    originalWP(2) = size(pathWPXBravo,2);
    originalWP(3) = size(pathWPXCharlie,2);
    originalWP(4) = size(pathWPXDelta,2);
    originalWP(5) = size(pathWPXEcho,2);
    waypointTotal = zeros(roverNumber, 1); 
    waypointTotal(1) = length(pathWPXAlpha); 
    waypointTotal(2) = length(pathWPXBravo);
    waypointTotal(3) = length(pathWPXCharlie);
    waypointTotal(4) = length(pathWPXDelta);
    waypointTotal(5) = length(pathWPXEcho);
    %---------------------------------------------------------------------%
    
    
    %---------------------------------------------------------------------%
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
    % Plot map 
    axis([0 xBoundary 0 yBoundary])
    title('Rover Environment')
    xlabel('X Position (m)')
    ylabel('Y Position (m)')
    grid on 
    hold on
    % Plot environment objects
    plot(steepSlopeOne, 'FaceColor', 'red', 'FaceAlpha', 0.2)
    hold on
    plot(steepSlopeTwo, 'FaceColor', 'red', 'FaceAlpha', 0.2)
    plot(steepSlopeThree, 'FaceColor', 'red', 'FaceAlpha', 0.2)
    plot(steepSlopeFour, 'FaceColor', 'red', 'FaceAlpha', 0.2)
    plot(obstacleOneP, 'FaceColor', 'black', 'FaceAlpha', 0.8) 
    plot(obstacleTwoP, 'FaceColor', 'black', 'FaceAlpha', 0.8)
    plot(obstacleThreeP, 'FaceColor', 'black', 'FaceAlpha', 0.8)
    plot(obstacleFourP, 'FaceColor', 'black', 'FaceAlpha', 0.8)
    plot(rockField, 'FaceColor', 'green', 'FaceAlpha', 0.4)
    plot(pathWPXAlpha,pathWPYAlpha, '-k*','LineWidth',0.5)
    plot(pathWPXBravo,pathWPYBravo, '-k*','LineWidth',0.5)
    plot(pathWPXCharlie,pathWPYCharlie, '-k*','LineWidth',0.5)
    plot(pathWPXDelta,pathWPYDelta, '-k*','LineWidth',0.5)
    plot(pathWPXEcho,pathWPYEcho, '-k*','LineWidth',0.5)
    th = 0:pi/50:2*pi;
    xObs1Rad = obsSafeRadius * cos(th) + obs(1,1);
    yObs1Rad = obsSafeRadius * sin(th) + obs(2,1);
    plot(xObs1Rad,yObs1Rad, 'b');
    xObs2Rad = obsSafeRadius * cos(th) + obs(1,2);
    yObs2Rad = obsSafeRadius * sin(th) + obs(2,2);
    plot(xObs2Rad,yObs2Rad, 'b');
    xObs3Rad = obsSafeRadius * cos(th) + obs(1,3);
    yObs3Rad = obsSafeRadius * sin(th) + obs(2,3);
    plot(xObs3Rad,yObs3Rad, 'b');
    xObs4Rad = obsSafeRadius * cos(th) + obs(1,4);
    yObs4Rad = obsSafeRadius * sin(th) + obs(2,4);
    plot(xObs4Rad,yObs4Rad, 'b');
    xObs5Rad = obsSafeRadius * cos(th) + obs(1,5);
    yObs5Rad = obsSafeRadius * sin(th) + obs(2,5);
    plot(xObs5Rad,yObs5Rad, 'b');
    xObs6Rad = obsSafeRadius * cos(th) + obs(1,6);
    yObs6Rad = obsSafeRadius * sin(th) + obs(2,6);
    plot(xObs6Rad,yObs6Rad, 'b');
    xObs7Rad = obsSafeRadius * cos(th) + obs(1,7);
    yObs7Rad = obsSafeRadius * sin(th) + obs(2,7);
    plot(xObs7Rad,yObs7Rad, 'b');
    xObs8Rad = obsSafeRadius * cos(th) + obs(1,8);
    yObs8Rad = obsSafeRadius * sin(th) + obs(2,8);
    plot(xObs8Rad,yObs8Rad, 'b');
    xObs9Rad = obsSafeRadius * cos(th) + obs(1,9);
    yObs9Rad = obsSafeRadius * sin(th) + obs(2,9);
    plot(xObs9Rad,yObs9Rad, 'b');
    xObs10Rad = obsSafeRadius * cos(th) + obs(1,10);
    yObs10Rad = obsSafeRadius * sin(th) + obs(2,10);
    plot(xObs10Rad,yObs10Rad, 'b');
    plot(obs(1,:),obs(2,:), 'o','MarkerSize',5, 'MarkerFaceColor',[0.75, 0, 0.75]);
    % Path Animation Initialisation 
    pathA = animatedline(startXAlpha,startYAlpha,'Color','red','LineWidth',0.85 );
    pathB = animatedline(startXBravo,startYBravo,'Color','red','LineWidth',0.85 );
    pathC = animatedline(startXCharlie,startYCharlie,'Color','red','LineWidth',0.85 );
    pathD = animatedline(startXDelta,startYDelta,'Color','red','LineWidth',0.85 );
    pathE = animatedline(startXEcho,startYEcho,'Color','red','LineWidth',0.85 );
%---------------------------------------------------------------------%


%---------------------------------------------------------------------%
    %%% Online Path Following - Dynamic Segment
     for time = 0:stepSize:endTime
        for i = 1:1:roverNumber
            %----------------------------------%
            if i == 1
                xoRef = xoARef;
                xdotRef = xdotARef;
                pathWPX = pathWPXAlpha;
                pathWPY = pathWPYAlpha;
                flagArrayRef = flagArrayARef;
                xo = xoA;
                xdot = xdotA;
                flagArray = flagArrayA;
                name = 'Alpha';
            elseif i == 2
                xoRef = xoBRef;
                xdotRef = xdotBRef;
                pathWPX = pathWPXBravo;
                pathWPY = pathWPYBravo;
                flagArrayRef = flagArrayBRef;
                xo = xoB;
                xdot = xdotB;
                flagArray = flagArrayB;
                name = 'Bravo';
            elseif i == 3
                xoRef = xoCRef;
                xdotRef = xdotCRef;
                pathWPX = pathWPXCharlie;
                pathWPY = pathWPYCharlie;
                flagArrayRef = flagArrayCRef;
                xo = xoC;
                xdot = xdotC;
                flagArray = flagArrayC;
                name = 'Charlie';
            elseif i == 4
                xoRef = xoDRef;
                xdotRef = xdotDRef;
                pathWPX = pathWPXDelta;
                pathWPY = pathWPYDelta;
                flagArrayRef = flagArrayDRef;
                xo = xoD;
                xdot = xdotD;
                flagArray = flagArrayD;
                name = 'Delta';
            elseif i == 5
                xoRef = xoERef;
                xdotRef = xdotERef;
                pathWPX = pathWPXEcho;
                pathWPY = pathWPYEcho;
                flagArrayRef = flagArrayERef;
                xo = xoE;
                xdot = xdotE;
                flagArray = flagArrayE;
                name = 'Echo';
            end
            %----------------------------------%


            %----------------------------------%
            %%% Rover Models
            if crash(i) == 0 && pathComplete(i) == 0 
            %%% Alpha Rover Ideal Model 
                xPreviousRef = xCurrentRef(i);       
                yPreviousRef = yCurrentRef(i); 
                resultantVelocityRef = sqrt((xoRef(1,1)^2 + (xoRef(2,1)^2)));
                xoRef(24,1) = resultantVelocityRef;
                psiCurrentRef = xoRef(12,1);
                xCurrentRef(i) = xoRef(7,1);        
                yCurrentRef(i) = xoRef(8,1);
                roverPathLengthRef(i) = roverPathLengthRef(i) + sqrt((xCurrentRef(i) - xPreviousRef)^2+(yCurrentRef(i) - yPreviousRef)^2); 

                %%% LOS Navigation Section
                    xDeltaWPRef = pathWPX(waypointCounterRef(i))-xCurrentRef(i);
                    yDeltaWPRef = pathWPY(waypointCounterRef(i))-yCurrentRef(i);
                    wpDistanceRef = sqrt((yDeltaWPRef)^2+(xDeltaWPRef)^2);

                    % Define Desired Velocity
                    VeRef = environmentalVelocity(xCurrent(i),yCurrent(i),rockField); 
                    desiredVelocityRef = VeRef - (slow(i)*0.03); 
                    if desiredVelocityRef < 0
                        desiredVelocityRef = 0.01;
                    end
                    
                
                    if isempty(obs) == 0 
                        [waypointCounterRef(i),wpDistanceRef] = obsWPCheck(obs,pathWPX,pathWPY, waypointCounterRef(i), wpDistanceRef, obsNumber, xCurrentRef(i), yCurrentRef(i));
                    end         
                    obsPresent = 0;
                    if isempty(obsRov) == 0 
                        [waypointCounterRef(i),wpDistanceRef] = obsWPCheck(obsRov,pathWPX,pathWPY, waypointCounterRef(i), wpDistanceRef, obsRovNumber, xCurrentRef(i), yCurrentRef(i));
                    end              
                    
                    % Check if within acceptance radius 
                    if (wpDistanceRef < acceptanceRadius)
                        waypointCounterRef(i) = waypointCounterRef(i) + 1;
                        if (waypointCounterRef(i) > size(pathWPX))
                             flagArrayRef(i) = 1; 
                             waypointCounterRef(i) = waypointCounterRef(i) - 1;
                        end
                    end

                    % Calculating LOS angle
                    psiWPRef = atan2(yDeltaWPRef, xDeltaWPRef);

                    % Call static obstacle avoidance function
                    flagArrayRef(2) = 0;
                    if isempty(obs) == 0 
                        [psiWPRef,flagArrayRef(2),flagArrayRef(3)] = StaticObstacleAvoidance(psiWPRef,obs, xCurrentRef(i), yCurrentRef(i), psiCurrentRef,obsSafeRadius,safetyFactor);
                    end 
                    
                    % Check for static rovers as obstacles 
                    if isempty(obsRov) == 0
                        [psiWPRef,flagArrayRef(2),flagArrayRef(3)] = StaticObstacleAvoidance(psiWPRef,obsRov, xCurrentRef(i), yCurrentRef(i), psiCurrentRef,rovSafeRadius,rovSafetyFactor);
                    end

                    % Mapping of Psi
                    [accumulateRef(i),stateRef(i)] = Psi_Mapper_Corrected(psiWPRef,psiLastRef(i), stateRef(i), xDeltaWPRef, yDeltaWPRef);
                    accumulationRef(i) = accumulationRef(i) + accumulateRef(i); 
                    if accumulationRef(i) >= 2*pi
                        accumulationRef(i) = accumulationRef(i) -2*pi;
                    end
                    psiLastRef(i) = psiWPRef;
                    [filteredPsiRef(i), dFilteredPsiRef(i)] = Psi_Filter(natFrequency, accumulationRef(i), filteredPsiRef(i), dFilteredPsiRef(i), stepSize);
                    psiRef = filteredPsiRef(i) - psiCurrentRef; 
                    [errorMappedPsiRef] = Psi_Mapper_ToPi(psiRef);
                %%% END of LOS Section 

                %%% Control Section
                    [psiCSRef,ePsiPreviousRef(i),ePsiIntegralRef(i)] = headingController(errorMappedPsiRef,ePsiIntegralRef(i),ePsiPreviousRef(i), stepSize);
                    [velCSRef,eVelPreviousRef(i),eVelIntegralRef(i)] = velocityController(desiredVelocityRef,resultantVelocityRef,eVelPreviousRef(i),eVelIntegralRef(i),stepSize);
                    controlSignalRef = [velCSRef; psiCSRef];
                %%% END of Control Section 

                %%% Derivative Section 
                    xdotARef = Rover_Model(xoRef, controlSignalRef);
                %%% End of Derivative Section 

                %%% Integral Section 
                    xoRef = rk4int('Rover_Model', stepSize, xoRef, controlSignalRef);
                %%% END of Integral Section 



            %%% Alpha Rover 'Realistic Model'
                    xPrevious = xCurrent(i);
                    yPrevious = yCurrent(i); 
                    resultantVelocity = sqrt((xo(1,1)^2 + (xo(2,1)^2)));
                    xo(24,1) = resultantVelocity;
                    if currentFaultMode(i) == 4
                        xo(12,1) = xo(12,1) + (1*pi/180); 
                    end
                    psiCurrent = xo(12,1);
                    xCurrent(i) = xo(7,1);
                    yCurrent(i) = xo(8,1);
                    roverPathLength(i) = roverPathLength(i) + sqrt((xCurrent(i) - xPrevious)^2+(yCurrent(i) - yPrevious)^2); 
 

                    %%% LOS Navigation Section
                        xDeltaWP = pathWPX(waypointCounter(i))-xCurrent(i);
                        yDeltaWP = pathWPY(waypointCounter(i))-yCurrent(i);
                        wpDistance = sqrt((yDeltaWP)^2+(xDeltaWP)^2);

                        % Define Desired Velocity 
                        Ve = environmentalVelocity(xCurrent(i),yCurrent(i),rockField); 
                        desiredVelocity = Ve - (slow(i)*0.03);    
                        if desiredVelocity < 0
                            desiredVelocity = 0.01;
                        end
                        
                        if isempty(obs) == 0 
                            [waypointCounter(i),wpDistance] = obsWPCheck(obs,pathWPX,pathWPY, waypointCounter(i), wpDistance, obsNumber, xCurrent(i), yCurrent(i));
                        end    
                        if isempty(obsRov) == 0 
                            [waypointCounter(i),wpDistance] = obsWPCheck(obsRov,pathWPX,pathWPY, waypointCounter(i), wpDistance, obsRovNumber, xCurrent(i), yCurrent(i));
                        end

                        % Check if within acceptance radius 
                        if (wpDistance <= acceptanceRadius)
                            waypointCounter(i) = waypointCounter(i) + 1;
                            if (waypointCounter(i) > size(pathWPX))
                                fprintf('%s final waypoint reached at time: %0.2f \n', name, time)
                                flagArray(1) = 1; 
                                pathComplete(i) = 1; 
                                waypointCounter(i) = waypointCounter(i) - 1;
                            end
                        end
                        
                        if waypointCounter(i) > originalWP(i) && enableTrafficControl(i) == 0
                            enableTrafficControl(i) = 1;
                            fprintf('Traffic control enabled on rover %s.\n',name)
                        end 
                        
                        % Calculating LOS angle
                        psiWP = atan2(yDeltaWP, xDeltaWP);

                        % Call static obstacle avoidance function
                        flagArray(2) = 0;
                        if isempty(obs) == 0 
                            [psiWP,flagArray(2),flagArray(3)] = StaticObstacleAvoidance(psiWP,obs, xCurrent(i), yCurrent(i), psiCurrent,obsSafeRadius,safetyFactor);
                        end 
                        
                        % Check for static rovers as obstacles 
                        if isempty(obsRov) == 0
                            [psiWP,flagArray(2),flagArray(3)] = StaticObstacleAvoidance(psiWP,obsRov, xCurrent(i), yCurrent(i), psiCurrent,rovSafeRadius,rovSafetyFactor);
                        end                      
                        

                        if flagArray(3) == 1
                            fprintf('Rover %s crashed at: %.2f s \n', name, time)
                            reconfigurationArray(i) = 1;
                            pathComplete(i) = 1;
                            reconfigurationArray(i) = 1; 
                            collisionType(i) = 1;
                        end

                        % Mapping of Psi
                        [accumulate(i),state(i)] = Psi_Mapper_Corrected(psiWP,psiLast(i), state(i), xDeltaWP, yDeltaWP);
                        accumulation(i) = accumulation(i) + accumulate(i); 
                        if accumulation(i) >= 2*pi
                            accumulation(i) = accumulation(i) -2*pi;
                        end
                        psiLast(i) = psiWP;
                        [filteredPsi(i), dFilteredPsi(i)] = Psi_Filter(natFrequency, accumulation(i), filteredPsi(i), dFilteredPsi(i), stepSize);
                        psi = filteredPsi(i) - psiCurrent; 
                        [errorMappedPsi] = Psi_Mapper_ToPi(psi);
                    %%% END of LOS Section 

                    %%% Control Section
                        [psiCS,ePsiPrevious(i),ePsiIntegral(i)] = headingController(errorMappedPsi,ePsiIntegral(i),ePsiPrevious(i), stepSize);
                        [velCS,eVelPrevious(i),eVelIntegral(i)] = velocityController(desiredVelocity,resultantVelocity,eVelPrevious(i),eVelIntegral(i),stepSize);
                        if currentFaultMode(i) == 1
                            velCS = 0;
                            psiCS = 0;
                        end
                        controlSignal = [velCS; psiCS];
                    %%% END of Control Section 

                    %%% Derivative Section 
                        xdot = Rover_Model_FDIR(xo, controlSignal, currentFaultMode(i));
                    %%% End of Derivative Section 

                    %%% Integral Section 
                        xo = rk4int_FDIR('Rover_Model_FDIR', stepSize, xo, controlSignal, currentFaultMode(i));
                    %%% END of Integral Section 
                        %
                    %%% FDIR Section 
                        % Inject fault at fault injection time 
                        if time == faultInjectionTime(i)
                            currentFaultMode(i) = faultMode(i);
                        end

                        % Fault Detection 
                        faultDetectedPrevious(i) = faultDetected(i);
                        if residualPosition(i) >= residualThreshold
                            faultDetected(i) = 1; 
                        end 
                        if (faultDetected(i) == 1) && (faultDetectedPrevious(i) ~= faultDetected(i))
                           fprintf('A fault was detected in rover %s at time %0.2f seconds. \n', name, time)
                           detectionTime(i) = time; 
                           psiResidualDetection = (psiCurrentRef - psiCurrent)*(180/pi); 
                           residualX = xCurrentRef(i) - xCurrent(i);
                           residualY = yCurrentRef(i) - yCurrent(i);
                           residualPositionDetection = sqrt((residualX)^2+(residualY)^2); 
                           xPositionDetection(i) = xCurrent(i);
                           yPositionDetection(i) = yCurrent(i); 
                        end

                        % Fault Isolation
                        if (time == detectionTime(i) + 1) && (detectionTime(i) ~= 0)
                           % Evaluate position residual 
                           psiResidualDiagnosis = (psiCurrentRef - psiCurrent)*(180/pi); 
                           residualX = xCurrentRef(i) - xCurrent(i);
                           residualY = yCurrentRef(i) - yCurrent(i);
                           residualPositionDiagnosis = sqrt((residualX)^2+(residualY)^2); 
                           
                           xPositionIsolation(i) = xCurrent(i);
                           yPositionIsolation(i) = yCurrent(i); 
                           xDiff = xPositionDetection(i) - xPositionIsolation(i);
                           yDiff = yPositionDetection(i) - yPositionIsolation(i);
                           roverMovementDiagnosis(i) = sqrt(xDiff^2 + yDiff^2);

                           psiResidualDelta = psiResidualDiagnosis - psiResidualDetection;
                           positionResidualDelta = residualPositionDiagnosis - residualPositionDetection;

                            if positionResidualDelta <= 0.01 
                                fprintf('Rover %s experienced an actuator fault. \n', name)
                                faultIsolated(i) = 1; 
                                faultModeIsolated(i) = 2; 
                                enableTrafficControl(i) = 1; 
                                fprintf('Traffic control enabled on rover %s.\n',name)
                            elseif abs(psiResidualDelta) <= 0.1 || roverMovementDiagnosis(i) <= (0.25*desiredVelocity)
                                fprintf('Rover %s experienced a power failure. \n', name)
                                faultIsolated(i) = 1;
                                faultModeIsolated(i) = 1; 
                                crash(i) = 1;
                                pathComplete(i) = 1;
                                reconfigurationArray(i) = 1;
                            else
                                fprintf('Rover %s experienced a heading sensor fault. \n', name)
                                faultIsolated(i) = 1; 
                                faultModeIsolated(i) = 4;
                                enableTrafficControl(i) = 1; 
                                fprintf('Traffic control enabled on rover %s.\n',name)
                            end
                        end
                        %
                    %%% END of FDIR Section    

                %%% Residual Generation 
                if mod(time, commsInterval)==0  
                    residualX = xCurrentRef(i) - xCurrent(i);
                    residualY = yCurrentRef(i) - yCurrent(i);
                    residualPosition(i) = sqrt((residualX)^2+(residualY)^2);
                    residualPsi(i) = (psiCurrentRef - psiCurrent)*(180/pi);
                %%% END of Trajectory Residual Generation 
                end
                
                % add halted rovers to 
                if pathComplete(i) == 1
                    obsRovX = [obsRovX xCurrent(i)];
                    obsRovY = [obsRovY yCurrent(i)];
                    obsRov = [obsRovX;obsRovY];
                    obsRovNumber = size(obsRovX); 
                end

            end

            %----------------------------------%

            %----------------------------------%
            % Restore values for next iteration
            if i == 1
                xoARef = xoRef;
                xdotARef = xdotRef;
                flagArrayARef = flagArrayRef;
                xoA = xo;
                xdotA = xdot;
            elseif i == 2
                xoBRef = xoRef;
                xdotBRef = xdotRef;
                flagArrayBRef = flagArrayRef;
                xoB = xo;
                xdotB = xdot;
            elseif i == 3
                xoCRef = xoRef;
                xdotCRef = xdotRef;
                flagArrayCRef = flagArrayRef;
                xoC = xo;
                xdotC = xdot;
            elseif i == 4
                xoDRef = xoRef;
                xdotDRef = xdotRef;
                flagArrayDRef = flagArrayRef;
                xoD = xo;
                xdotD = xdot;
            elseif i == 5
                xoERef = xoRef;
                xdotERef = xdotRef;
                flagArrayERef = flagArrayRef;
                xoE = xo;
                xdotE = xdot;
            end
            %----------------------------------%
        end

        % Plot Rovers' Paths
        if mod(time, commsInterval*100)== 0
            addpoints(pathA,xCurrent(1),yCurrent(1));
            drawnow 
            addpoints(pathB,xCurrent(2),yCurrent(2));
            drawnow
            addpoints(pathC,xCurrent(3),yCurrent(3));
            drawnow      
            addpoints(pathD,xCurrent(4),yCurrent(4));
            drawnow
            addpoints(pathE,xCurrent(5),yCurrent(5));
            drawnow         
        end 

        %----------------------------------%
        % Rover Collision Checker 
        slow = zeros(roverNumber,1); 
        for j = 1:1:roverNumber
            if crash(j) == 0 && enableTrafficControl(j) == 1
                for k = 1:1:roverNumber
                    if crash(k) == 0 
                        if j ~= k
                            pointDistance = sqrt((xCurrent(j)-xCurrent(k))^2 +(yCurrent(j)-yCurrent(k))^2);
                            if pointDistance <= 1.2  && pointDistance > 0.8    % Action to be taken if rovers within 1.2 m of eachother)
                                slow(j) = slow(j) + 1;
                            elseif pointDistance <= 0.8 && pointDistance > 0.35
                                slow(j) = slow(j) + 2;
                            elseif pointDistance <= 0.35 % Crash occurrance
                                crash(j) = 1;
                                pathComplete(j) = 1;
                                fprintf('%0.2f crashed into a rover at %0.2f s. \n', j, time)
                                collisionType(j) = 2; 
                                crash(k) = 1;
                                pathComplete(k) = 1;
                                fprintf('%0.2f crashed into a rover at %0.2f s \n',k, time)
                                collisionType(k) = 2;
                            end
                        end
                    end
                end 
            end 
        end 
        %----------------------------------%
        
        %----------------------------------%
        %%% Reconfiguration Method: Path Blending 
        distanceBetweenRovers = zeros(roverNumber,1);
        newWPX = [];
        newWPY = [];
        if reconfigMode == 1
            % Evaluate if any rover paths require path blending 
            if isempty(reconfigurationArray) == 0
                for n = 1:1:roverNumber
                    if reconfigurationArray(n) == 1
                        for j = 1:1:roverNumber
                            % Find distance between operating rovers and
                            % crashed rover 
                            if n ~= j && (crash(j) == 0)
                                distanceBetweenRovers(j) = (sqrt((xCurrent(n)-xCurrent(j))^2+(yCurrent(n)-yCurrent(j))^2));
                            else
                                distanceBetweenRovers(j) = 2000;       % Number large enough that it wont be selected 
                            end 
                        end
                        % Evaluate which rover is closest 
                        [minDistance, distanceIndex] = min(distanceBetweenRovers); 
                        if distanceIndex == 1  
                            fprintf('Path blending allocated to Alpha. \n')
                            WPArrayXClosestRover = pathWPXAlpha;
                            WPArrayYClosestRover = pathWPYAlpha;
                        elseif distanceIndex == 2
                            fprintf('Path blending allocated to Bravo. \n')
                            WPArrayXClosestRover = pathWPXBravo;
                            WPArrayYClosestRover = pathWPYBravo;
                        elseif distanceIndex == 3
                            fprintf('Path blending allocated to Charlie. \n')
                            WPArrayXClosestRover = pathWPXCharlie;
                            WPArrayYClosestRover = pathWPYCharlie;
                        elseif distanceIndex == 4
                            fprintf('Path blending allocated to Delta. \n')
                            WPArrayXClosestRover = pathWPXDelta;
                            WPArrayYClosestRover = pathWPYDelta;
                        elseif distanceIndex == 5
                            fprintf('Path blending allocated to Echo. \n')
                            WPArrayXClosestRover = pathWPXEcho;
                            WPArrayYClosestRover = pathWPYEcho;
                        end 
                        % Compare WP Number for crashed rover and chosen rover
                        waypointRemaining = zeros(2,1);
                        % Remaining WP crashed rover 
                        waypointRemaining(1) = waypointTotal(n) - waypointCounter(n);
                        % Remaining WP closest rover
                        waypointRemaining(2) = waypointTotal(distanceIndex) - waypointCounter(distanceIndex);
                        % Select rover with fewer remaining waypoints
                        [minWPRemain, waypointIndex] = min(waypointRemaining);
                        if waypointIndex == 1
                            % Set crashed rover as least WP remaining
                            currentWPNumber = waypointCounter(i);
                        else 
                            % Set closest rover as least WP remaining
                            currentWPNumber = waypointCounter(distanceIndex);
                        end 

                        % Set WP array of crashed rover
                        if n == 1  
                            WPArrayXCrashedRover = pathWPXAlpha;
                            WPArrayYCrashedRover = pathWPYAlpha;
                        elseif n == 2
                            WPArrayXCrashedRover = pathWPXBravo;
                            WPArrayYCrashedRover = pathWPYBravo;
                        elseif n == 3
                            WPArrayXCrashedRover = pathWPXCharlie;
                            WPArrayYCrashedRover = pathWPYCharlie;
                        elseif n == 4
                            WPArrayXCrashedRover = pathWPXDelta;
                            WPArrayYCrashedRover = pathWPYDelta;
                        elseif n == 5
                            WPArrayXCrashedRover = pathWPXEcho;
                            WPArrayYCrashedRover = pathWPYEcho;
                        end

                        % Set max wp to the wp total of whichever rover has
                        % fewer WP 
                        if (waypointTotal(n) >= waypointTotal(distanceIndex))
                            maxWP = waypointTotal(distanceIndex);
                        else
                            maxWP = waypointTotal(n);
                        end 

                        % Replace future WP with blended path WP 
                        for k = maxWP:-1:currentWPNumber
                            midpointX = (WPArrayXClosestRover(k) + WPArrayXCrashedRover(k))/2;
                            midpointY = (WPArrayYClosestRover(k) + WPArrayYCrashedRover(k))/2; 
                            % Check if new final point is too close to
                            % other points 
                            if k == maxWP 
                                targetSafe = 1; 
                                distanceBetweenTargets = zeros(roverNumber,1);
                                while targetSafe == 1
                                    for checkTargets = 1:1:roverNumber
                                        if checkTargets ~= n && checkTargets ~= j
                                            distanceBetweenTargets(checkTargets) = (sqrt((roverTargetsX(checkTargets)-midpointX)^2+(roverTargetsY(checkTargets)-midpointY)^2));
                                        end 
                                    end
                                    if min(distanceBetweenTargets(distanceBetweenTargets>0)) < 1 
                                        targetSafe = 1;
                                        midpointY = midpointY - 0.1;
                                    else
                                        targetSafe = 0;
                                    end 
                                end
                            end
                            newWPX = [newWPX midpointX];
                            newWPY = [newWPY midpointY];
                        end

                        % Add current rover position as intermediate WP to
                        % reduce reduce trajectory error
                        newWPX = [newWPX xCurrent(distanceIndex)];
                        newWPY = [newWPY yCurrent(distanceIndex)];

                        % Add previous WP of cloest rover to new WP array
                        for k = (currentWPNumber-2):-1:1
                            WPX = WPArrayXClosestRover(k);
                            WPY = WPArrayYClosestRover(k);
                            newWPX = [newWPX WPX];
                            newWPY = [newWPY WPY];
                        end

                        % Flip New Array & Allocate 
                        if distanceIndex == 1  
                            pathWPXAlpha = flip(newWPX);  
                            pathWPYAlpha = flip(newWPY);    
                        elseif distanceIndex == 2
                            pathWPXBravo = flip(newWPX);  
                            pathWPYBravo = flip(newWPY);
                        elseif distanceIndex == 3
                            pathWPXCharlie = flip(newWPX);  
                            pathWPYCharlie = flip(newWPY);    
                        elseif distanceIndex == 4
                            pathWPXDelta = flip(newWPX);  
                            pathWPYDelta = flip(newWPY);
                        elseif distanceIndex == 5
                            pathWPXEcho = flip(newWPX);  
                            pathWPYEcho = flip(newWPY);
                        end 
                        % Plot New Path 
                        hold all 
                        plot(newWPX,newWPY, '-b*','LineWidth',0.25) 
                        % Mark path as reconfigured
                        reconfigurationArray(n) = 0; 
                        enableTrafficControl(n) = 1;
                        pathBlendedSelection = n; 
                        fprintf('Traffic control enabled on rover %0.2f.\n',distanceIndex)
                    end
                end
            end
        end
        %----------------------------------%
        
        
        %----------------------------------%
        %%% Reconfiguration Method: Target Re-allocation
        % Re-release previously allocated targets of crashed rover 
        if reconfigMode == 0 && isempty(crash) == 0
            for k = 1:1:roverNumber
                if crash(k) ~= pathEnd(k)   % check only newly crashed rovers 
                    targetIndex = (targetReallocation == k);
                    reconfigurationArray(targetIndex) = k; 
                end
            end 

            % Re-allocate targets
            etaAllocationArray = zeros(roverNumber,1); 
            if isempty(reconfigurationArray) == 0
                for k = 1:1:roverNumber 
                    if reconfigurationArray(k) == 1
                        % Find eta of each free rover
                        for j = 1:1:roverNumber
                            if j == 1
                                etaAllocationArray(j) = 10000;
                            elseif k ~= j && (crash(j) == 0)
                                timeToNewTarget = (sqrt((roverTargetsX(k)-roverTargetsX(j))^2+(roverTargetsY(k)-roverTargetsY(j))^2))/0.1; 
                                etaAllocationArray(j) = etaArray(j) + timeToNewTarget;  
                            else
                                etaAllocationArray(j) = 10000;       % Number large enough that it wont be selected 
                            end 
                        end
                        % Selected best rover for reallocation 
                        [minEta, etaIndex] = min(etaAllocationArray); 
                        % Add reallocated target as extra WP 
                        if etaIndex == 1 
                            pathWPXAlpha = [pathWPXAlpha roverTargetsX(k)];
                            pathWPYAlpha = [pathWPYAlpha roverTargetsY(k)]; 
                            fprintf('Target point was re-allocated to Alpha. \n')
                            allocatedTo = 1; 
                        elseif etaIndex == 2
                            pathWPXBravo = [pathWPXBravo roverTargetsX(k)];
                            pathWPYBravo = [pathWPYBravo roverTargetsY(k)];
                            fprintf('Target point was re-allocated to Bravo. \n')
                            allocatedTo = 2; 
                        elseif etaIndex == 3
                            pathWPXCharlie = [pathWPXCharlie roverTargetsX(k)];
                            pathWPYCharlie = [pathWPYCharlie roverTargetsY(k)];
                            fprintf('Target point was re-allocated to Charlie. \n')
                            allocatedTo = 3; 
                        elseif etaIndex == 4
                            pathWPXDelta = [pathWPXDelta roverTargetsX(k)];
                            pathWPYDelta = [pathWPYDelta roverTargetsY(k)];
                            fprintf('Target point was re-allocated to Delta. \n')
                            allocatedTo = 4; 
                        elseif etaIndex == 5
                            pathWPXEcho = [pathWPXEcho roverTargetsX(k)];
                            pathWPYEcho = [pathWPYEcho roverTargetsY(k)];
                            fprintf('Target point was re-allocated to Echo. \n')
                            allocatedTo = 5; 
                        end 
                        targetReallocation(k) = etaIndex;  
                        % Mark point as reallocated
                        reconfigurationArray(k) = 0;
                        
                    end 
                end
            end 
        end
        %----------------------------------%
        
        
        %----------------------------------%
        % Display a message if a target has been captured 
        
        for targetAcquire = 1:1:roverNumber
            for roverCheck = 1:1:roverNumber
                xDiff = xCurrent(roverCheck) - roverTargetsX(targetAcquire);
                yDiff = yCurrent(roverCheck) - roverTargetsY(targetAcquire);
                acquisitionDistance = sqrt((xDiff)^2 + (yDiff)^2);
                if acquisitionDistance <= acceptanceRadius && targetAcquired(targetAcquire) == 0
                    fprintf('Rover %0.2f acquired target %0.2f. \n', roverCheck, targetAcquire)
                    targetAcquired(targetAcquire) = 1;
                    acquiredBy(targetAcquire) = roverCheck; 
                end
            end   
        end 
        
        %----------------------------------%
        
        
        %----------------------------------%
        if isempty(obsRov) == 0 
            for drawRover = 1:1:roverNumber
                if crash(drawRover) == 1 && rovObsDrawn(drawRover) == 0
                    xObsRoverRad = rovSafeRadius * cos(th) + xCurrent(drawRover);
                    yObsRoverRad = rovSafeRadius * sin(th) + yCurrent(drawRover);
                    plot(xObsRoverRad,yObsRoverRad, 'b');
                    rovObsDrawn(drawRover) = 1;
                end
            end
        end 
        %----------------------------------%

        
        %----------------------------------%
        % End simulation if all rovers finish traversal
        if (crash(1) == 1) || (pathComplete(1) == 1) 
            pathEnd(1) = 1; 
        end 
        if (crash(2) == 1) || (pathComplete(2) == 1) 
            pathEnd(2) = 1; 
        end 
        if (crash(3) == 1) || (pathComplete(3) == 1) 
            pathEnd(3) = 1; 
        end 
        if (crash(4) == 1) || (pathComplete(4) == 1) 
            pathEnd(4) = 1; 
        end 
        if (crash(5) == 1) || (pathComplete(5) == 1) 
            pathEnd(5) = 1; 
        end 
        if (pathEnd(1)==1) && (pathEnd(2)==1) && (pathEnd(3)==1) && (pathEnd(4)==1) && (pathEnd(5)==1)
            fprintf('Simultation Complete: All rovers have finished travelling.\n')
            break
        end
        if all(targetAcquired) 
            fprintf('Simultation Complete: All rovers have finished travelling.\n')
            break
        end
        %----------------------------------%

    end
     
     
    
     fprintf('Test run %0.2f complete \n', testRun)
     faultInRover(testRun) = faultyRover;
     faultInjectionTimeData(testRun) = faultInjectionTimeArray(currentInjectionTime);
     faultInjectionType(testRun) = 1;
     targetAllocatedTo(testRun) = allocatedTo; 
     targetAcquiredBy(testRun,1) = acquiredBy(1); 
     targetAcquiredBy(testRun,2) = acquiredBy(2); 
     targetAcquiredBy(testRun,3) = acquiredBy(3);  
     targetAcquiredBy(testRun,4) = acquiredBy(4); 
     targetAcquiredBy(testRun,5) = acquiredBy(5);
     %pathBlendedWith(testRun) = pathBlendedSelection;
     trafficControl(testRun,1) = enableTrafficControl(1);
     trafficControl(testRun,2) = enableTrafficControl(2);
     trafficControl(testRun,3) = enableTrafficControl(3);
     trafficControl(testRun,4) = enableTrafficControl(4);
     trafficControl(testRun,5) = enableTrafficControl(5);
     collisionData(testRun,1) = collisionType(1);
     collisionData(testRun,2) = collisionType(2);
     collisionData(testRun,3) = collisionType(3);
     collisionData(testRun,4) = collisionType(4);
     collisionData(testRun,5) = collisionType(5);
 end
    %---------------------------------------------------------------------%

% filename = 'SingleRoverFaultTests_Set1_FM4.xlsx';
% writematrix(testNo,filename,'Sheet',1,'Range','B2:B126')
% writematrix(faultInRover,filename,'Sheet',1,'Range','C2:C126')
% writematrix(faultInjectionTimeData,filename,'Sheet',1,'Range','D2:D126')
% writematrix(faultInjectionType,filename,'Sheet',1,'Range','E2:E126')
% %writematrix(pathBlendedWith,filename,'Sheet',1,'Range','F2:F126') 
% writematrix(targetAcquiredBy,filename,'Sheet',1,'Range','H2:L126') 
% writematrix(trafficControl,filename,'Sheet',1,'Range','N2:R126') 
% writematrix(collisionData,filename,'Sheet',1,'Range','T2:X126') 
