function [environmentalVelocity] = environmentalVelocity(coordX,coordY,rockField)
% This function looks up desire environmental velocity of a rover 
% Cost values take into account the maxStep allowed for a move to a new
% node. This ensures that terrain cost will be reasonable in proportion to
% distance cost. 
%---------------------------------------------------------------------%


% Look up terrain section of point
if isinterior(rockField,coordX,coordY)
    environmentalVelocity = 0.05;
else 
    environmentalVelocity = 0.1;
end

end

