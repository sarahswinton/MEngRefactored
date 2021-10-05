function [psiCS,ePsiPrevious,ePsiIntegral] = headingController(errorMappedPsi,ePsiIntegral,ePsiPrevious,stepSize)
% This function carries out calculations as the PID heading controller.
psiKp = 45;    
psiKi = 3;     
psiKd = 0.75;   

ePsi = errorMappedPsi;
ePsiIntegral = ePsiIntegral + ePsi*stepSize;
ePsiDerivative = (ePsi - ePsiPrevious)/stepSize;
psiCS = (psiKp * ePsi)  + (psiKi * ePsiIntegral) + (psiKd * ePsiDerivative);
ePsiPrevious = ePsi;
end

