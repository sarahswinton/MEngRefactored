function [psiNow,obstacleAvoid,crashOccur] = StaticObstacleAvoidance(psiNow,obs, xCurrent, yCurrent, psiCurrent,safeRadius,safetyFactor)
% This function will evaluate whether an 
% If an obstacle is present, a new value of PsiNow will be passed back
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Initialising values: 
obsCounter = 1; 
obstacleDetected = 0; 
obstacleAvoid = 0; 
obsSafeRadius = safeRadius; 
[~, obsColumnNumber] = size(obs);
crashOccur = 0;
psiWP = psiNow;  
%---------------------------------------------------------------------%


%---------------------------------------------------------------------%
while (obsCounter <= obsColumnNumber)
    % Find Range to Current Obstacle 
    xDeltaObs = obs(1,obsCounter)-xCurrent;
    yDeltaObs = obs(2,obsCounter)-yCurrent;
    obsRange = sqrt((xDeltaObs)^2+(yDeltaObs)^2);

    % Initiate crash sequence if safety radius is comprimised
    % But ensure rovers aren't checking for crashes with themselves when
    % they finish their path i.e. have reached acceptance radius for target
    if (obsRange <= obsSafeRadius) && obsRange >= 0.175
        %fprintf('Rover has crashed into an obstacle. \n')
        crashOccur = 1;
    end 

    % Find angles between the rover and obstacle
    psiObs =  atan2(yDeltaObs,xDeltaObs);
    visionAngle = abs(psiNow-psiObs);
    % Ensure [-pi,pi] boundary visionAngle issues are resolved
    if abs(psiNow) >= (0.95*pi)
        visionAngle = abs(abs(psiNow)-abs(psiObs));
    end

    % Detect if objects are 'visible'
    if (obsRange <= 3)  && (visionAngle <= 1.0472) % i.e. only adjust heading if in danger of collision
        obstacleVisible = 1;
    end

    % Take evasive action if obstacle is on path
    if obsRange <= 1 && (visionAngle <=0.610865) % +- 35 degrees
          obstacleAvoid = 1;
          if (psiCurrent <= psiObs) && (psiWP < pi/2) && (psiWP >= 0)
            psiNow = psiObs - atan((obsSafeRadius*safetyFactor)/obsRange);
          elseif (psiCurrent > psiObs) && (psiWP < pi/2) && (psiWP >= 0)
            psiNow = psiObs + atan((obsSafeRadius*safetyFactor)/obsRange);
          elseif (psiCurrent <= psiObs) && (psiWP >= pi/2) && (psiWP < pi)
            psiNow = psiObs - atan((obsSafeRadius*safetyFactor)/obsRange);
          elseif (psiCurrent > psiObs) && (psiWP >= pi/2) && (psiWP < pi)
            psiNow = psiObs + atan((obsSafeRadius*safetyFactor)/obsRange);
          elseif (psiCurrent <= psiObs) && (psiWP >= pi)
            psiNow = psiObs - atan((obsSafeRadius*safetyFactor)/obsRange);
          elseif (psiCurrent > psiObs) && (psiWP >= pi)
            psiNow = psiObs + atan((obsSafeRadius*safetyFactor)/obsRange);
          elseif (psiCurrent <= psiObs) && (psiWP >= -pi/2) && (psiWP < 0) 
            psiNow = psiObs - atan((obsSafeRadius*safetyFactor)/obsRange);
          elseif (psiCurrent > psiObs) && (psiWP >= -pi/2) && (psiWP < 0) 
            psiNow = psiObs + atan((obsSafeRadius*safetyFactor)/obsRange);
          elseif (psiCurrent <= psiObs) && (psiWP < -pi/2)&& (psiObs < -pi/2)
            psiNow = psiObs - atan((obsSafeRadius*safetyFactor)/obsRange);
          elseif (psiCurrent > psiObs) && (psiWP < -pi/2) && (psiObs < -pi/2) 
            psiNow = psiObs + atan((obsSafeRadius*safetyFactor)/obsRange);
           elseif (psiCurrent <= psiObs) && (psiWP < -pi/2) && (sign(psiObs) ~= sign(psiWP)) && visionAngle <= 0.349066
            psiNow = psiNow - atan((obsSafeRadius*(1/visionAngle))/obsRange);     
          end
    end
 % Increment Counter 
 obsCounter = obsCounter + 1; 
end
end 


