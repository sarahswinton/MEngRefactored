% Made by: S. Shilliday
% Created: 30/10/2018
% Modified by: Maria Dias Neves (2312034D)
% Last Edited: 12/07/2020
% 
%
% Inputs: natural frequency, reference input, previous filtered input, 
%         and previous filtered 1st order derivative.
% *************************************************************************

function [psif, dpsif] = Psi_Filter(natfreq, psides, psif, dpsif, dt)

zeta = 1;       % Damping ratio (we want it to be critically damped).

ddpsif = (-2*zeta*natfreq*dpsif) - ((natfreq^2)*psif) + ((natfreq^2)*psides);

dpsif_old = dpsif;
dpsif = dpsif + dt*ddpsif;
psif = psif + dt*dpsif_old;