function [accumulate,state] = Psi_Mapper_Corrected(psiWP,psiLast, state, xDelta, yDelta)
% This function maps Psi from [-pi, pi] to [-inf,inf]

% WP in First Quadrant (Unity Circle) 
if (xDelta >= 0 && yDelta >= 0)
    if(state==3) % use Guide Rule if in quadrant 3
        if ((psiWP + abs(psiLast)) <= pi)
            accumulate = psiWP - psiLast;
        else
            accumulate = psiWP - psiLast - 2*pi;
        end
    else 
        accumulate = psiWP - psiLast;
    end
    state = 1;

%WP in Second Quadrant (Unity Circle) 
elseif (xDelta < 0 && yDelta >= 0)
    if(state==4) 
        if ((psiWP + abs(psiLast)) <= pi)
            accumulate = psiWP - psiLast;
        else
            accumulate = psiLast - psiWP - 2*pi; % potentially needs to be +ve
        end
    elseif (state == 3)
        accumulate = psiWP - psiLast + 2*pi;
    elseif (state==2) 
        accumulate = psiWP - psiLast;
    else 
        accumulate = psiWP - psiLast;
    end
    state = 2;

%WP in Third Quadrant (Unity Circle) 
elseif (xDelta < 0 && yDelta < 0)
    if(state==1) 
        if ((abs(psiWP) + psiLast) <= pi)
            accumulate = psiWP - psiLast;
        else
            accumulate = psiWP - psiLast + 2*pi; % potentially +ve
        end
    elseif (state==2)
        accumulate = psiWP - psiLast + 2*pi;
    else
        accumulate = psiWP - psiLast;
    end
    state = 3;
    
%WP in Fourth Quadrant (Unity Circle) 
else % (xDelta >= 0 && yDelta < 0)
    if(state==2) 
        if ((abs(psiWP) + psiLast) <= pi)
            accumulate = psiWP - psiLast;
        else
            accumulate = psiWP - psiLast + 2*pi;
        end
    else 
        accumulate = psiWP - psiLast;
    end
    state = 4;
end
