function [distanceToPath] = trajectoryTracker(currentXWP,currentYWP, previousXWP,previousYWP, currentX, currentY)
% This function output the distance of the rover from the line between its
% current and previous waypoints. 

distToPreviousWP = sqrt((currentX-previousXWP)^2+(currentY-previousYWP)^2);
distToCurrentWP = sqrt((currentX-currentXWP)^2+(currentY-currentYWP)^2); 
distBetweenWP = sqrt((previousXWP-currentXWP)^2+(previousYWP-currentYWP)^2);
% if ((distanceBetweenWP)-(distToCurrentWP)-(distToPreviousWP)) == 0
%     distanceToPath = 0;
% else
%     distanceToPath = sqrt(distToPreviousWP^2 - (((distBetweenWP^2)-(distToCurrentWP^2)+(distToPreviousWP^2))/(2*distBetweenWP)));
% end
% Area of scalene triange 
s = (distToPreviousWP + distToCurrentWP + distBetweenWP)/2;
triangleArea = sqrt(s*(s-distToPreviousWP)*(s-distToCurrentWP)*(s-distBetweenWP));
% Distance to path is trianlge height 
distanceToPath = 2*(triangleArea/distBetweenWP);
end

