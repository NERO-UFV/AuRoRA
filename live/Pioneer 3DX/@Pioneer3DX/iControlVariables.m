function iControlVariables(p3dx)

% Pionner P3DX
% ========================================================================
% Robot pose
p3dx.pPos.X    = zeros(12,1); % Current pose (point of control)
p3dx.pPos.Xa   = zeros(12,1); % Past pose

p3dx.pPos.Xc   = zeros(12,1); % Current pose (center of the robot)
p3dx.pPos.Xp   = zeros(12,1); % Current pose (computed by the robot)

p3dx.pPos.Xd   = zeros(12,1); % Desired pose
p3dx.pPos.Xda  = zeros(12,1); % Past desired pose

p3dx.pPos.Xr   = zeros(12,1); % Reference pose
p3dx.pPos.Xra  = zeros(12,1); % Past reference pose

% First time derivative 
p3dx.pPos.dX   = zeros(12,1); % Current pose
p3dx.pPos.dXd  = zeros(12,1); % Desired pose
p3dx.pPos.dXr  = zeros(12,1); % Reference pose

% Pose error
p3dx.pPos.Xtil = p3dx.pPos.Xd - p3dx.pPos.X; 

% ========================================================================
% Sensor data
p3dx.pPos.Xso  = zeros(12,1); % Initial sensor data
p3dx.pPos.Xs   = zeros(12,1); % Current sensor data 
p3dx.pPos.Xsa  = zeros(12,1); % Past sensor data
p3dx.pPos.dXs  = zeros(12,1); % First time derivative of sensor data

% Sensorial fusion data
p3dx.pPos.Xf   = zeros(12,1);

% GPS data: Latitude, Longitude e Altitude
p3dx.pPos.Xg  = zeros(3,1);

% Control Signal== ======================================================
% Linear and Angular Velocity
p3dx.pSC.U   = [0;0]; % Current
p3dx.pSC.Ua  = [0;0]; % Past
p3dx.pSC.Ud  = [0;0]; % Desired
p3dx.pSC.Uda = [0;0]; % Past desired
p3dx.pSC.Ur  = [0;0]; % Reference
p3dx.pSC.Kinematics_control = 0;

% Linear and Angular Acceleration
p3dx.pSC.dU   = [0;0]; % Current
p3dx.pSC.dUd  = [0;0]; % Desired