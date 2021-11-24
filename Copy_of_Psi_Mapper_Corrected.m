function [accumulate,state] = Copy_of_Psi_Mapper_Corrected(psiNow,psiLast, state, xDelta, yDelta)
% This function maps Psi from [-pi, pi] to [-inf,inf]

% WP in First Quadrant (Unity Circle) 
if (xDelta >= 0 && yDelta >= 0)
    if(state==3) 
        if ((psiNow + abs(psiLast)) <= pi)
            accumulate = psiNow - psiLast;
        else
            accumulate = psiLast - psiNow + 2*pi;
        end
    else 
        accumulate = psiNow - psiLast;
    end
    state = 1;

%WP in Second Quadrant (Unity Circle) 
elseif (xDelta >= 0 && yDelta < 0)
    if(state==4) 
        if ((abs(psiNow) + psiLast) <= pi)
            accumulate = psiNow - psiLast;
        else
            accumulate = psiNow - psiLast + 2*pi;
        end
    else 
        accumulate = psiNow - psiLast;
    end
    state = 2;

%WP in Third Quadrant (Unity Circle) 
elseif (xDelta < 0 && yDelta < 0)
    if(state==1) 
        if ((abs(psiNow) + psiLast) <= pi)
            accumulate = psiNow - psiLast;
        else
            accumulate = psiNow - psiLast + 2*pi; % potentially +ve
        end
    elseif (state==4)
        accumulate = psiNow - psiLast + 2*pi;
    else
        accumulate = psiNow - psiLast;
    end
    state = 3;
    
%WP in Fourth Quadrant (Unity Circle) 
elseif (xDelta < 0 && yDelta >= 0)
    if (state == 2)
        if (psiNow + abs(psiLast) <= pi)
            accumulate = psiNow - psiLast;
        else 
            accumulate = psiLast - psiNow + 2*pi;
        end
    elseif (state == 3)
        accumulate = psiNow - psiLast - 2*pi; % potentially needs to be +ve
    else 
        accumulate = psiNow - psiLast;
    end
    state = 4;
end
