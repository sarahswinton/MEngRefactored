function [desiredPsi] = Psi_Mapper_ToPi(psi)
% Map from [-inf,inf] to [-pi, pi]
if(sign(psi)==1)
    n = floor(psi/pi);
elseif(sign(psi)==-1)
    n = ceil(psi/pi);
else
    n = 0;
end 
remainder = rem(n,2);
if(remainder==0)
    psiMapped = psi - n*pi;
else 
    if(sign(remainder)==1)
        psiMapped = psi - (n+1)*pi;
    else 
        psiMapped = psi - (n-1)*pi;
    end
end

desiredPsi = psiMapped;
end

