function iControlVariables(load)

load.pPos.X  = zeros(12,1);    % Real Pose
load.pPos.Xa = zeros(12,1);    % Previous Pose

load.pPos.Xd   = zeros(12,1);    % Desired Pose
load.pPos.Xda  = zeros(12,1);    % Previous Desired Pose
load.pPos.dXd  = zeros(12,1);    % Desired first derivative Pose 

load.pPos.Xtil = zeros(12,1);    % Posture Error

load.pPos.dX   = zeros(12,1);    % First derivative

load.pSC.U     = zeros(4,1);     % Control Signal
load.pSC.Ur    = zeros(4,1);     % Reference kinematic control signal 
load.pSC.Ud    = zeros(4,1);     % Desired Control Signal (sent to robot)

load.pSC.Wd    = zeros(4,1);     % Desired rotor velocity;
load.pSC.Xr    = zeros(12,1);    % Reference pose
