function iControlVariables(pendulum)

pendulum.pPos.Q    = zeros(8,1);    % Real Pose
pendulum.pPos.dQ   = zeros(8,1);    % First derivative
pendulum.pPos.ddQ  = zeros(8,1);    % Second derivative
pendulum.pPos.Qa   = zeros(8,1);    % Previous Pose
pendulum.pPos.dQa  = zeros(8,1);    % Previous first derivative
% pendulum.pPos.Qo   = zeros(8,1);    % Bias pose: Calibration

pendulum.pPos.Qd   = zeros(8,1);    % Desired Pose
pendulum.pPos.dQd  = zeros(8,1);    % Desired pose first derivative  
pendulum.pPos.ddQd  = zeros(8,1);   % Desired pose second derivative  
pendulum.pPos.Qda  = zeros(8,1);    % Previous Desired Pose
pendulum.pPos.Qtil = zeros(8,1);    % Posture Error

pendulum.pSC.Qr    = zeros(8,1);    % Reference pose
pendulum.pSC.dQr   = zeros(8,1);    % Reference pose's first derivative
pendulum.pSC.Qa    = zeros(8,1);    % Previous reference pose
pendulum.pSC.dQa   = zeros(8,1);    % Previous pose's first derivative
pendulum.pSC.U     = zeros(4,1);    % Control Signal
pendulum.pSC.Ur    = zeros(4,1);    % Reference kinematic control signal 
pendulum.pSC.Ud    = zeros(4,1);    % Desired Control Signal (sent to robot)

pendulum.pSC.Wd    = zeros(4,1);    % Desired rotor velocity;
pendulum.pSC.D     = zeros(6,1);    % Disturbance Vector

% Pendulum's cartesian coordinates
pendulum.pPos.Xp = zeros(3,1);


    

