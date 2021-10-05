% Adapted from ENG3036 Lab File
% May need to change xo back to x 
function xo = rk4int(modelname, h, x, u, fault)
% This function performs one 4th Order Runge-Kutta integration step
% h is xo (i.e. the xcurr matrix)
% x is the step size
% u is the control signal
k1 = h*feval(modelname, x, u, fault);          % evaluate derivative k1
k2 = h*feval(modelname, x+k1/2, u, fault);     % evaluate derivative k2
k3 = h*feval(modelname, x+k2/2, u, fault);     % evaluate derivative k3
k4 = h*feval(modelname, x+k3, u, fault);		% evaluate derivative k4

xo = x + (k1 + 2*k2 + 2*k3 + k4)/6;		% averaged output
