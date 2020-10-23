function mInit(obj)
    % Posture variables 
    % Robot Variables
    obj.pPos.X    = zeros(9,1);  % real pose [ x1 y1 z1 ; x2 y2 z2 ; x3 y3 z3 ]
    obj.pPos.Xd   = zeros(9,1);  % reference pose [ x1 y1 z1 ; x2 y2 z2 ; x3 y3 z3 ]
    obj.pPos.Xr   = zeros(9,1);  % reference pose [ x1 y1 z1 ; x2 y2 z2 ; x3 y3 z3 ]
    obj.pPos.dX   = zeros(9,1);  % real pose first derivative
    obj.pPos.dXd  = zeros(9,1);  % desired pose first derivative
    obj.pPos.dXr  = zeros(9,1);  % desired pose first derivative
    
    % Formation Variable
    obj.pPos.Q    = zeros(6,1);  % formation variables [ xF yF psi pF qF beta ]
    obj.pPos.Qr   = zeros(6,1);  % reference formation
    obj.pPos.Qd   = zeros(6,1);  % desired formation   
    obj.pPos.Qtil = zeros(6,1);  % formation error
    obj.pPos.dQr  = zeros(6,1);  % reference formation first derivative
    obj.pPos.dQd  = zeros(6,1);  % desired formation first derivative

    % Control Signal variables
    obj.pSC.Ur = zeros(12,1);     % reference control signal (sent to dynamic compensator)
    obj.pSC.Ud = zeros(12,1);     % desired control signal (sent to robot)

    % Parameters variables
    obj.pPar.K1 = 1*diag([0.8 0.8 0.5 0.02 0.015 0.03]);    % kinematic control gain  - controls amplitude
    obj.pPar.K2 = 1*diag([0.1 0.1 0.5 0.5 0.5 0.5]);        % kinematic control gain - control saturation
    
    % Auxiliary Parameters
    obj.pSeq = 0;
    obj.pID = 0;
    obj.pRobots = zeros(3,1);
end

