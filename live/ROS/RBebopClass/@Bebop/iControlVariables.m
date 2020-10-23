
function iControlVariables(obj)

    obj.pPos.X    = zeros(12,1);    % Real Pose
    obj.pPos.Xa   = zeros(12,1);    % Previous Pose
    obj.pPos.Xo   = zeros(12,1);    % Bias pose: Calibration

    obj.pPos.X(3) = 0.75;           % Start Altitude [m] 

    obj.pPos.Xd   = zeros(12,1);    % Desired Pose
    obj.pPos.Xda  = zeros(12,1);    % Previous Desired Pose
    obj.pPos.dXd  = zeros(12,1);    % Desired first derivative Pose 

    % Pose reference
    obj.pPos.Xr  = zeros(12,1); 
    obj.pPos.Xra = zeros(12,1); 
    
    obj.pPos.Xtil = zeros(12,1);    % Posture Error

    obj.pPos.dX   = zeros(12,1);    % First derivative

    obj.pSC.U     = zeros(4,1);     % Control Signal
    obj.pSC.Ur    = zeros(4,1);     % Reference kinematic control signal 
    obj.pSC.Ud    = zeros(6,1);     % Desired Control Signal (sent to robot)

    obj.pSC.Wd    = zeros(4,1);     % Desired rotor velocity;
    obj.pSC.Xr    = zeros(12,1);    % Reference pose
    obj.pSC.Kinematics_control = 0; % Flag telling that there is a kinematic controller in another control function (0=NO, 1=YES)
    obj.pSC.Control_flag = 0;       % Flag used to activate or deactivate some control tasks


    obj.pSC.tcontrole = tic;
    obj.pSC.D     = zeros(6,1);     % Disturbance Vector
    
end
