function [xOut,pathLength,crashType,roverEndTime] = ChildRoverFcn(pathWPX,pathWPY, startX, startY, obs)
%---------------------------------------------------------------------%
% 2243716 MEng Project 
% Fault Tolerant Coordination of Multiple Rovers for Planetary Exploration
% Adapted from Maria Dias Neves 2312034D Summer Research Project
% LOS Mapping adapted from Breivik [2003]
% Child Rover Model 
%clear;      % clear Workspace
%clc;        % clear Command Window
%---------------------------------------------------------------------%


%%%%% Initial Segment 
%---------------------------------------------------------------------%
stepSize = 0.01;            % Step Size may be adjusted
commsInterval = 0.02;       % Communication Interval 
endTime = 800;               % Length of simulation time (will change)
counter = 0;                % Counter to store data
plotVoltage = [];           % Voltage variables
plotVoltageLeft = 0; 
plotVoltageRight = 0; 
% Initialise state variables and derivatives
xo = [0;0;0;0;0;0;startX;startY;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
xdot = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; 
% Initialise control errors
eVelocity = 0;      ePsi = 0;
eVelPrevious = 0;   ePsiPrevious = 0;
eVelIntegral = 0;   ePsiIntegral = 0;
eVelDerivative = 0; ePsiDerivative = 0;
% Initialise control gains
velKp = 3;          psiKp = 45;    
velKi = 13.75;      psiKi = 3;     
velKd = 0;          psiKd = 0.75;     
% Initialise heading filter values
natFrequency = 2.9;     % 2.9 from Maria's Project    % check up on this
filteredPsi = 0;
dFilteredPsi = 0; 
% Initialise guidance variables: 
WP(1,:) = [pathWPX];      %[0,5,10,10,5,0,-5,-5,0];       % WP X values
WP(2,:) = [pathWPY];      %[0,0,5,10,15,15,10,5,0];       % WP Y values
[~, WPColumnNumber] = size(WP);
acceptanceRadius = 0.175;           % 1/2 robot length
waypointCounter = 1;
% Initialise heading angle mapping variables: (refer to Dr Breivik's Thesis)
state = 0;              % tracks which unity circle quadrant from previous time step
psiLast = 0;            % angle from previous time step
psiNow = 0;             % angle from current time step 
accumulate = 0;         % angular change this time step
accumulation = 0;       % total angular change so far
% Initialise obstacle avoidance variables:
safeRadius = 0.2;       % Safe radius to avoid obstacle 
safetyFactor = 2;
obstacleDetected = 0; 
obsAvoidPsi = 0;
crashOccur = 0;
%[~, obsColumnNumber] = size(obs);
% Initialise plotting variables 
pathLength = 0; 
xCurrent = 0; 
yCurrent = 0; 
pathEnd = 0;
obsPresent = 0; 
crashType = 0;
roverEndTime = 0;

rockFieldX = [15 15 24 24]; 
rockFieldY = [10 15 18 13];
rockField = polyshape(rockFieldX,rockFieldY);
%---------------------------------------------------------------------%

%%%%% Dynamic Segment 
%---------------------------------------------------------------------%
for time = 0:stepSize:endTime
    xPrevious = xCurrent;
    yPrevious = yCurrent; 
    % Find resultant velocity
    resultantVelocity = sqrt((xo(1,1)^2 + (xo(2,1)^2)));
    xo(24,1) = resultantVelocity;
    % Current Heading 
    psiCurrent = xo(12,1);
    % Current Position
    xCurrent = xo(7,1);
    yCurrent = xo(8,1);

    % Calculate distance moved in previous step: 
    xDelta = xCurrent - xPrevious; 
    yDelta = yCurrent - yPrevious; 
    pathLengthIncrement = sqrt((yDelta)^2+(xDelta)^2);
    pathLength = pathLength + pathLengthIncrement; 

    % At commsInterval, store data
    if mod(time, commsInterval)==0
        counter = counter + 1; 
        timeOut(counter) = time; 
        xOut(:, counter) = xo;
        xdotOut(:, counter) = xdot; 
        plotVoltage(counter,1) = plotVoltageRight;
        plotVoltage(counter,2) = plotVoltageLeft;
    end   


    %%% LOS Navigation Section
        xDeltaWP = WP(1,waypointCounter)-xCurrent;
        yDeltaWP = WP(2,waypointCounter)-yCurrent;
        wpDistance = sqrt((yDeltaWP)^2+(xDeltaWP)^2);

        % Define Desired Velocity
        Ve = environmentalVelocity(xCurrent,yCurrent,rockField); 
        desiredVelocity = Ve;
        obsNumber = 10;
        obsPresent = 0; 
        if isempty(obs) == 0 
        % Check if the current waypoint is close to any obstacles 
            for obsWPCheck = 1:1:obsNumber 
                xWPObs = obs(1,obsWPCheck) - pathWPX(waypointCounter);
                yWPObs = obs(2,obsWPCheck) - pathWPY(waypointCounter);
                distWPObs = sqrt((yWPObs)^2+(xWPObs)^2);
                if distWPObs <= 0.2 && wpDistance <= 0.3
                    obsPresent = 1; 
                end 
            end 
            
            if obsPresent == 1
                waypointCounter = waypointCounter + 1; 
                % Recalculate the WP distance if an obstacle is present 
                xDeltaWP = WP(1,waypointCounter)-xCurrent;
                yDeltaWP = WP(2,waypointCounter)-yCurrent;
                wpDistance = sqrt((yDeltaWP)^2+(xDeltaWP)^2);
            end
        end         
        
        % Check if within acceptance radius 
        if (wpDistance < acceptanceRadius)
            waypointCounter = waypointCounter + 1;
            if (waypointCounter > WPColumnNumber)
                fprintf('Final Waypoint Reached at time: %0.2f \n', time)
                roverEndTime = time; 
                pathEnd = 1;
                crashType = 0;
                break
            end
        end
        
        % Calculating LOS angle
        psiNow = atan2(yDeltaWP, xDeltaWP);

        if isempty(obs) == 0 
        % Call static obstacle avoidance function
        [psiNow,obstacleDetected,crashOccur] = StaticObstacleAvoidance(psiNow,obs, xCurrent, yCurrent, psiCurrent,safeRadius,safetyFactor);
        end 

        if (crashOccur == 1)
            pathEnd = 1;
            fprintf('Rover crashed at: %.2f s \n', time) 
            roverEndTime = time; 
            crashType = 1;
            break
        end
    
        % Map Psi from [-pi,pi] to [-inf,inf]
        [accumulate,state] = Psi_Mapper_Corrected(psiNow,psiLast, state, xDeltaWP, yDeltaWP);

        % Update accumulation variable and psiLast
        accumulation = accumulation + accumulate; 
        if accumulation >= 2*pi
            accumulation = accumulation -2*pi;
        end
        psiLast = psiNow;

        % Filtering the response of Psi  
        [filteredPsi, dFilteredPsi] = Psi_Filter(natFrequency, accumulation, filteredPsi, dFilteredPsi, stepSize);
        psi = filteredPsi - psiCurrent; 

        % Map from [-inf,inf] to [-pi, pi]
        [errorMappedPsi] = Psi_Mapper_ToPi(psi);

    %%% END of LOS Section 

    %%% Control Section
        ePsi = errorMappedPsi;
        ePsiIntegral = ePsiIntegral + ePsi*stepSize;
        ePsiDerivative = (ePsi - ePsiPrevious)/stepSize;
        psiCS = (psiKp * ePsi)  + (psiKi * ePsiIntegral) + (psiKd * ePsiDerivative);
        ePsiPrevious = ePsi;

        % Resultant Velocity Controller (PI)
        eVelocity = desiredVelocity - resultantVelocity;
        eVelIntegral = eVelIntegral + eVelocity*stepSize;
        eVelDerivative = (eVelocity - eVelPrevious)/stepSize;
        velCS = (velKp * eVelocity)  + (velKi * eVelIntegral) + (velKd * eVelDerivative);
        eVelPrevious = eVelocity;

        % Define control signal matrix
        controlSignal = [velCS; psiCS];
        plotVoltageLeft = velCS + psiCS;
        plotVoltageRight = velCS - psiCS;
    %%% END of Control Section 

    %%% Derivative Section 

        % Call Rover_model
        xdot = Rover_Model(xo, controlSignal);
    %%% End of Derivative Section 

    %%% Integral Section 
        % Call integrator model 
        xo = rk4int('Rover_Model', stepSize, xo, controlSignal);
    %%% END of Integral Section 
end
%---------------------------------------------------------------------%
end

