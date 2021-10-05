function [waypointCounter,wpDistance] = obsWPCheck(obs,pathWPXAlpha,pathWPYAlpha, waypointCounter, wpDistance, obsNumber, xCurrent, yCurrent)
% This function will increment to the next waypoint if the current waypoint
% is deemed 'too close' to an obstacle.
obsPresent  = 0;
for obsWPCheck = 1:1:obsNumber
    xWPObs = obs(1,obsWPCheck) - pathWPXAlpha(waypointCounter);
    yWPObs = obs(2,obsWPCheck) - pathWPYAlpha(waypointCounter);
    distWPObs = sqrt((yWPObs)^2+(xWPObs)^2);
    if distWPObs <= 0.2 && wpDistance <= 0.3
        obsPresent = 1; 
    end 
end 

if obsPresent == 1
    waypointCounter = waypointCounter + 1; 
    % Recalculate the WP distance if an obstacle is present 
    xDeltaWP = pathWPXAlpha(waypointCounter)-xCurrent;
    yDeltaWP = pathWPYAlpha(waypointCounter)-yCurrent;
    wpDistance = sqrt((yDeltaWP)^2+(xDeltaWP)^2);
end

end

