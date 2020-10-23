function iControlVariables(drone)

drone.pPos.X    = zeros(12,1);    % Real Pose
drone.pPos.Xa   = zeros(12,1);    % Previous Pose
drone.pPos.Xo   = zeros(12,1);    % Bias pose: Calibration

drone.pPos.X(3) = 0.75;           % Start Altitude [m] 
 
drone.pPos.Xd   = zeros(12,1);    % Desired Pose
drone.pPos.Xda  = zeros(12,1);    % Previous Desired Pose
drone.pPos.dXd  = zeros(12,1);    % Desired first derivative Pose 

% Pose reference
drone.pPos.Xr  = zeros(12,1); 
drone.pPos.Xra = zeros(12,1); 

drone.pPos.Xtil = zeros(12,1);    % Posture Error
drone.pPos.intXtil = zeros(3,1);  % Integral do erro (Experimentos Daniel Dourado)

drone.pPos.dX   = zeros(12,1);    % First derivative

drone.pSC.U     = zeros(4,1);     % Control Signal
drone.pSC.Ur    = zeros(4,1);     % Reference kinematic control signal 
drone.pSC.dUr   = zeros(4,1);     % Reference dynamic control signal
drone.pSC.dUrK  = ones(2,1);      % Acceleration gain for roll and pitch
drone.pSC.Ud    = zeros(4,1);     % Desired Control Signal (sent to robot)

drone.pSC.Wd    = zeros(4,1);     % Desired rotor velocity;
drone.pSC.Xr    = zeros(12,1);    % Reference pose
drone.pSC.Kinematics_control = 0; % Flag telling that there is a kinematic controller in another control function (0=NO, 1=YES)
drone.pSC.Control_flag = 0;       % Flag used to activate or deactivate some control tasks


drone.pSC.tcontrole = tic;
drone.pSC.D     = zeros(6,1);     % Disturbance Vector

