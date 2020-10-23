function tInit(trif)
    % Robots posture variables
    trif.pPos.X = zeros(9,1);
    trif.pPos.Xd = zeros(9,1);
    trif.pPos.Xr = zeros(9,1);
    trif.pPos.dX = zeros(9,1);
    trif.pPos.dXr = zeros(9,1);
    
    % Formation ID
    trif.pID = 0;
    
    % Formation posture variables
    trif.pPos.Q = zeros(9,1);
    trif.pPos.Qr = zeros(9,1);
    trif.pPos.Qd = zeros(9,1);
    trif.pPos.QdA = zeros(9,1);
    trif.pPos.Qtil = zeros(9,1);
    trif.pPos.dQ = zeros(9,1);
    trif.pPos.dQr = zeros(9,1);
    trif.pPos.dQd = zeros(9,1);
    
    % Control signal variables
    trif.pSC.Ur = zeros(8,1);
    trif.pSC.Ud = zeros(8,1);
    
    % Parameters variables
%     trif.pPar.K1 = .8*diag([.6  .6  1   1  4 2.2 2.2 2.2 1]);
    trif.pPar.K1 = .8*diag([.6  .6  1   1  1.5 1.5 1.5 1.5 1]);
    % trif.pPar.K1 = .8*diag([.6  .6  1   1.5   2.2 2.2 2.2 2.2 1.5]);
    trif.pPar.K2 =  1*diag([1   1   1   1     1   1   1   1   1]);
    trif.pPar.ti = tic;
    trif.pPar.R = {0 0 0; 'T' 'T' 'T'};
end