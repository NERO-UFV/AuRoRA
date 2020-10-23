function iControlVariables(drone)

drone.pPos.X    = zeros(12,1);    % Real Pose
drone.pPos.Xa   = zeros(12,1);    % Previous Pose
drone.pPos.Xo   = zeros(12,1);    % Bias pose: Calibration

drone.pPos.X(3) = 0.75;           % Start Altitude [m] 
 
drone.pPos.Xd   = zeros(12,1);    % Desired Pose
drone.pPos.Xda  = zeros(12,1);    % Previous Desired Pose
drone.pPos.dXd  = zeros(12,1);    % Desired first derivative Pose 

drone.pPos.Xtil = zeros(12,1);    % Posture Error

drone.pPos.dX   = zeros(12,1);    % First derivative

drone.pSC.U     = zeros(4,1);     % Control Signal
drone.pSC.Ur    = zeros(4,1);     % Reference kinematic control signal 
drone.pSC.Ud    = zeros(4,1);     % Desired Control Signal (sent to robot)

drone.pSC.Wd    = zeros(4,1);     % Desired rotor velocity;
drone.pSC.Xr    = zeros(12,1);    % Reference pose

drone.pSC.D     = zeros(6,1);     % Disturbance Vector

