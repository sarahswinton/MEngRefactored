function [velCS,eVelPrevious,eVelIntegral] = velocityController(desiredVelocity,resultantVelocity,eVelPrevious,eVelIntegral,stepSize)
% This function carries out calculations as the PI velocity controller.
velKp = 3;         
velKi = 13.75;      
velKd = 0;          

eVelocity = desiredVelocity - resultantVelocity;
eVelIntegral = eVelIntegral + eVelocity*stepSize;
eVelDerivative = (eVelocity - eVelPrevious)/stepSize;
velCS = (velKp * eVelocity)  + (velKi * eVelIntegral) + (velKd * eVelDerivative);
eVelPrevious = eVelocity;
end

