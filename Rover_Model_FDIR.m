% Origin: 
    % Rover_Motor_Model_v1.m
    % Rover_Rigid_Body_Model_v1.m
% From Stuart Shilliday (PGR) 
% Further Adaptations By Maria Dias Neves (2312034D)
% For use in 2243716S MEng Project 2020

%---------------------------------------------------------------------%

% Defining States
% u = xcur(1);
% v = xcur(2);
% w = xcur(3);
% p = xcur(4);
% q = xcur(5);
% r = xcur(6);
% x = xcur(7);  
% y = xcur(8);
% z = xcur(9);
% phi = xcur(10);
% theta = xcur(11);
% psi = xcur(12);
% i_lf = xcur(13);      
% i_lb = xcur(14);     
% i_rf = xcur(15);      
% i_rb = xcur(16);      
% Omega_lf = xcur(17);  
% Omega_lb = xcur(18);  
% Omega_rf = xcur(19);  
% Omega_rb = xcur(20);  
% fz = xcur(21);      % fz = unmatched(1,1);  %heave = fz
% tx = xcur(22);      % tx = unmatched(2,1);  %roll = tx
% ty = xcur(23);      % ty = unmatched(3,1);  %pitch = ty

% Defining Outputs 
% xdot(1,1) = udot;
% xdot(2,1) = vdot;
% xdot(3,1) = wdot;
% xdot(4,1) = pdot;
% xdot(5,1) = qdot;
% xdot(6,1) = rdot;
% xdot(7,1) = xxdot;
% xdot(8,1) = ydot;
% xdot(9,1) = zdot;
% xdot(10,1) = phidot;
% xdot(11,1) = thetadot;
% xdot(12,1) = psidot;
% xdot(13,1) = idot(1,1);
% xdot(14,1) = idot(2,1);
% xdot(15,1) = idot(3,1);
% xdot(16,1) = idot(4,1);
% xdot(17,1) = Omegadot(1,1);
% xdot(18,1) = Omegadot(2,1);
% xdot(19,1) = Omegadot(3,1);
% xdot(20,1) = Omegadot(4,1);
% xdot(21,1) = 0;     % fzdot = 0 because fz = 0
% xdot(22,1) = 0;     % txdot = 0 because tx = 0
% xdot(23,1) = 0;     % tydot = 0 because ty = 0
% xdot(24,1) = 0;     

%---------------------------------------------------------------------%

function xdot = Rover_Model_FDIR(xcur, controlSignal, faultMode)

%---------------------------------------------------------------------%
% Motor Modelling
% Initialisation 
b = 0.008;          % Viscous torque [N m]
Jm = 0.005;         % Moment of inertia of motor [kg m^2]
Kt = 0.35;          % Torque constant [N m A^-1]
Ke = 0.35;          % EMF constant [V rad^-1 s^-1]
L = 0.1;            % Inductance of circuit [H]
R = 4;              % Resistance of circuit [ohms]
alpha = -0.133;     % Gradient for efficiency curve [A^-1]
gamma = 0.6;        % Offset for efficiency curve
zeta = 0.002;       % Base friction on wheel

idot = zeros(4,1);      % Initialise rate of change of current matrix
Omegadot = zeros(4,1);  % Initialise rate of change of rotational velocity matrix
eta = zeros(4,1);       % Initialise efficiency matrix
torques = zeros(4,1);   % Initialise torque matrix
V=zeros(4,1);           % Initialise voltage matrix

% Calculate Voltage In Each Motor 
% control_signal = [cs_vel; cs_psi];  
V(1,1) = (controlSignal(1,1) + controlSignal(2,1));  % Front LHS V
V(2,1) = (controlSignal(1,1) + controlSignal(2,1));  % Back LHS V
V(3,1) = (controlSignal(1,1) - controlSignal(2,1));  % Front RHS V 
V(4,1) = (controlSignal(1,1) - controlSignal(2,1));  % Back RHS V

% Motor Model
for j = 1:4
    
    idot(j) = (V(j) - (R*xcur(j+12)) - (Ke*xcur(j+16)))/L;
    Omegadot(j) = ((Kt*xcur(j+12)) - (b*xcur(j+16)) - (zeta*xcur(j+16)))/Jm;
    
    eta(j) = (alpha*xcur(j+12)) + gamma;
    torques(j) = Kt * xcur(j+12) * eta(j);
    
end

% Inject Actuator Faults
if faultMode == 2
    torques(1) = torques(1)*0.5;
    torques(2) = torques(2)*0.5;
elseif faultMode == 3
    torques(3) = torques(3)*0.5;
    torques(4) = torques(4)*0.5;
elseif faultMode == 1 
    torques(1) = 0;
    torques(2) = 0;
    torques(3) = 0;
    torques(4) = 0;
end

    
% Assign r.o.c. of I and Omega to output array
xdot(13,1) = idot(1,1);
xdot(14,1) = idot(2,1);
xdot(15,1) = idot(3,1);
xdot(16,1) = idot(4,1);
xdot(17,1) = Omegadot(1,1);
xdot(18,1) = Omegadot(2,1);
xdot(19,1) = Omegadot(3,1);
xdot(20,1) = Omegadot(4,1);

%---------------------------------------------------------------------%
% Rigid Body Model


%--------------------%
% Used to keep psi between -180 -> +180
if(xcur(12)>=(pi))
   xcur(12)=xcur(12)-(2*pi);
elseif(xcur(12)<-pi)
   xcur(12)=xcur(12)+(2*pi);
end
%--------------------%

%--------------------%
% States
u = xcur(1);
v = xcur(2);
w = xcur(3);
p = xcur(4);
q = xcur(5);
r = xcur(6);
x = xcur(7);  
y = xcur(8);
z = xcur(9);
phi = xcur(10);
theta = xcur(11);
psi = xcur(12);
i_lf = xcur(13);      
i_lb = xcur(14);     
i_rf = xcur(15);      
i_rb = xcur(16);      
Omega_lf = xcur(17);  
Omega_lb = xcur(18);  
Omega_rf = xcur(19);  
Omega_rb = xcur(20); 
fz = xcur(21);          % Heave
tx = xcur(22);          % Roll
ty = xcur(23);          % Pitch
% Since only 2D motion is considered; Heave, Roll and Pitch remain zero.
%--------------------%

%---------------------------------------------------------------------%
% Setup Cosines and Sines
% Cosines
cphi = cos(phi); ctheta = cos(theta); cpsi = cos(psi);

% Sines
sphi = sin(phi); stheta = sin(theta); spsi = sin(psi);
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Robot Specifications
m = 2.148;          % Mass of robot, kg
wheel_r = 0.0635;   % Radius of Wheel, m
x_area = 0.0316;    % Area presented on the x-axis, m.m
y_area = 0.0448;    % Area presented on the y-axis, m.m
Jx = 0.0140;        % Moment of Inertia about the x-axis, kg.m.m
Jy = 0.0252;        % Moment of Inertia about the y-axis, kg.m.m
Jz = 0.0334;        % Moment of Inertia about the z-axis, kg.m.m
mr = 0.1245;        % Moment Arm

% Constants
g = 9.81;           % Gravity, m/s.s
Cd = 0.89;          % Drag Coefficent
rho= 1.29;          % Air density
W = 21.0719;        % m*g; %Weight
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Input Forces
% Calculated from the torques generated by the wheels. 
force_l1 = (torques(1)/wheel_r);
force_l2 = (torques(2)/wheel_r);
force_r1 = (torques(3)/wheel_r);
force_r2 = (torques(4)/wheel_r);
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Slip
% calculates the slip angle for each wheel
bottom = sqrt((u^2)+(v^2));   % calculate denominator

if bottom == 0                    % check if 0,
    beta = 0;                     %   if 0 then no movement, no slip
else
    beta = asin(v/bottom);        % else calculate slip
end

if (abs(beta) > pi)
    beta =  0;
end
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Propulsion Forces
surge = (force_l1+force_l2+force_r1+force_r2)*cos(beta);
sway = (force_l1+force_l2+force_r1+force_r2)*sin(beta);
heave = fz;
roll = tx;
pitch = ty;
yaw = ((force_l1+force_l2)-(force_r1+force_r2))*mr;
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Dampening Terms
% fric_k = 0.35;
% fric_m = 0.44;
% fric_x = 0.22;
% fric_n = 0.18;
% fric_y = 1;
% fric_z = 0.3;

% Friction
Fx_fric = 4.6358*u;     % W*fric_x*u;
Fy_fric = 21.0719*v;    % W*fric_y*v;
Fz_fric = 6.3216*w;     % W*fric_z*w;
K_fric = 0.9182*p;      % W*fric_k*mr*p;
M_fric = 1.1543*q;      % W*fric_m*mr*q;
N_fric = 0.4722*r;      % W*fric_n*mr*r;

% Air Resistance
% x-axis
Fx_ar = 0.0181*u*abs(u); % 0.5*Cd*x_area*rho*u*abs(u);

% Drag due to faulty actuators
Fx_drag = Fx_fric * 0;
Fy_drag = Fy_fric * 0;

% Total Dampening
X_damp = Fx_fric+Fx_ar + Fx_drag;
Y_damp = Fy_fric + Fy_drag;
Z_damp = Fz_fric;
K_damp = K_fric;
M_damp = M_fric;
N_damp = N_fric;
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Gravity Terms
X_grav = W*stheta;
Y_grav = W*sphi*ctheta;
Z_grav = (W*ctheta*cphi)-W;
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Forces and Torques
% Forces
X = surge-X_damp+X_grav;
Y = sway-Y_damp+Y_grav;
Z = heave-Z_damp+Z_grav;

% Torques
K = roll-K_damp;
M = pitch-M_damp;
N = yaw-N_damp;
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Equations of Motion
% Linear Accelerations
udot = (X/m)+(v*r)-(w*q);
vdot = (Y/m)+(w*p)-(u*r);
wdot = (Z/m)+(u*q)-(v*p);

% Rotational Accelerations
pdot = (K-((Jz-Jy)*q*r))/Jx;
qdot = (M-((Jx-Jz)*r*p))/Jy;
rdot = (N-((Jy-Jx)*p*q))/Jz;
 %---------------------------------------------------------------------%
 %---------------------------------------------------------------------%
% Kinematics
% Linear Kinematics
xxdot = ((cpsi*ctheta)*u)+(((-spsi*cphi)+(cpsi*stheta*sphi))*v)+(((spsi*sphi)+(cpsi*stheta*cphi))*w); 
    % xxdot because xdot is the output matrix
ydot = ((spsi*ctheta)*u)+(((cpsi*cphi)+(spsi*stheta*sphi))*v)+(((-sphi*cpsi)+(spsi*cphi*stheta))*w);
zdot = ((-stheta)*u)+((ctheta*sphi)*v)+((ctheta*cphi)*w);


% Angular Kinematics
% In theory +/-90 degrees for pitch is undefined but matlab tan()
% gives it a figure. Also this situation should not occur.
ttheta = tan(theta);
phidot = p+((sphi*ttheta)*q)+((cphi*ttheta)*r); 
thetadot = ((cphi)*q)+((-sphi)*r);
psidot = ((sphi/ctheta)*q)+((cphi/ctheta)*r);
%---------------------------------------------------------------------%

%---------------------------------------------------------------------%
% Assign variables to output
xdot(1,1) = udot;
xdot(2,1) = vdot;
xdot(3,1) = wdot;
xdot(4,1) = pdot;
xdot(5,1) = qdot;
xdot(6,1) = rdot;
xdot(7,1) = xxdot;
xdot(8,1) = ydot;
xdot(9,1) = zdot;
xdot(10,1) = phidot;
xdot(11,1) = thetadot;
xdot(12,1) = psidot;
xdot(21,1) = 0;     % fzdot = 0 because fz = 0
xdot(22,1) = 0;     % txdot = 0 because tx = 0
xdot(23,1) = 0;     % tydot = 0 because ty = 0
xdot(24,1) = 0;     

%---------------------------------------------------------------------%
end 

