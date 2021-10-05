function [xCoordOne, yCoordOne, xCoordTwo,yCoordTwo] = PPStepCorrector(xOutOneX,xOutOneY, xOutTwoX, xOutTwoY)
% This function adjusts the length of each coordinate array so they have
% the same length.

stepCountOne = size(xOutOneX,2);
stepCountTwo = size(xOutTwoX,2);
xCoordOne = xOutOneX;
yCoordOne = xOutOneY;
xCoordTwo = xOutTwoX;
yCoordTwo = xOutTwoY;

% Correct elements in array 
if stepCountOne > stepCountTwo
    endXPointB = xCoordTwo(end);
    endYPointB = yCoordTwo(end); 
    for additionalSteps = stepCountTwo:1:(stepCountOne-1)
        xCoordTwo = [xCoordTwo endXPointB];
        yCoordTwo = [yCoordTwo endYPointB];
    end  
elseif stepCountOne < stepCountTwo
    endXPointA = xCoordOne(end);
    endYPointA = yCoordOne(end); 
    for additionalSteps = stepCountOne:1:(stepCountTwo-1)
        xCoordOne = [xCoordOne endXPointA];
        yCoordOne = [yCoordOne endYPointA];
    end
end 

end

