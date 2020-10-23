function fInit(linef)
    % Robots posture variables
    linef.pPos.X = zeros(6,1);
    linef.pPos.Xr = zeros(6,1);
    linef.pPos.dX = zeros(6,1);
    linef.pPos.dXr = zeros(6,1);
    
    % Formation ID
    linef.pID = 0;
    
    % Formation posture variables
    linef.pPos.Q = zeros(6,1);
    linef.pPos.Qr = zeros(6,1);
    linef.pPos.Qd = zeros(6,1);
    linef.pPos.Qtil = zeros(6,1);
    linef.pPos.dQ = zeros(6,1);
    linef.pPos.dQr = zeros(6,1);
    linef.pPos.dQd = zeros(6,1);
    
    % Control signal variables
    linef.pSC.Ur = zeros(8,1);
    linef.pSC.Ud = zeros(8,1);
    
    % Parameters variables
    linef.pPar.K1 = 1*diag([1 1 1 1 1 1]);
    linef.pPar.K2 = 1*diag([1 1 1 1 1 1]);
end