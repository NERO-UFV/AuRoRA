function tFormationControl(trif)
    % Direct transformation
    trif.tDirTrans;        % Get formation variables
    
    % Formation pose error
    trif.tFormationError;
    
    % Inverse Jacobian Matrix parameters {Formation vector: [xf yf zf rf alphaf deltaf]}
    thetaf = trif.pPos.Q(4);
    phif   = trif.pPos.Q(5);
    psif   = trif.pPos.Q(6);
    pf     = trif.pPos.Q(7);
    qf     = trif.pPos.Q(8);
    betaf  = trif.pPos.Q(9);
    
    J.Inv.I = [1 0 0;
               0 1 0;
               0 0 1];
     
    J.Inv.O = [0 0 0;
               0 0 0;
               0 0 0];
    
    J.Inv.pPar{1} = [pf*cos(thetaf)*cos(psif)-pf*sin(thetaf)*sin(phif)*sin(psif) pf*cos(thetaf)*cos(phif)*sin(psif)  -pf*sin(thetaf)*sin(psif)+pf*cos(thetaf)*sin(phif)*cos(psif);
                     pf*cos(thetaf)*sin(psif)+pf*sin(thetaf)*sin(phif)*cos(psif) -pf*cos(thetaf)*cos(phif)*cos(psif) pf*sin(thetaf)*cos(psif)+pf*cos(thetaf)*sin(phif)*sin(psif);
                     -pf*sin(thetaf)*cos(phif)                                   -pf*cos(thetaf)*sin(phif)           0];
                 
    J.Inv.pPar{2} = [sin(thetaf)*cos(psif)+cos(thetaf)*sin(phif)*sin(psif) 0 0;
                     sin(thetaf)*sin(psif)-cos(thetaf)*sin(phif)*cos(psif) 0 0;
                     cos(thetaf)*cos(phif)                                 0 0];
                 
    J.Inv.pPar{3} = [qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif) qf*cos(thetaf+betaf)*cos(phif)*sin(psif)  -qf*sin(thetaf+betaf)*sin(psif)+qf*cos(thetaf+betaf)*sin(phif)*cos(psif);
                     qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif) -qf*cos(thetaf+betaf)*cos(phif)*cos(psif) qf*sin(thetaf+betaf)*cos(psif)+qf*cos(thetaf+betaf)*sin(phif)*sin(psif);
                     -qf*sin(thetaf+betaf)*cos(phif)                                         -qf*cos(thetaf+betaf)*sin(phif)           0];
                 
    J.Inv.pPar{4} = [0 sin(thetaf+betaf)*cos(psif)+cos(thetaf+betaf)*sin(phif)*sin(psif) qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif);
                     0 sin(thetaf+betaf)*sin(psif)-cos(thetaf+betaf)*sin(phif)*cos(psif) qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif);
                     0 cos(thetaf+betaf)*cos(phif)                                       -qf*sin(thetaf+betaf)*cos(phif)];
    
    % Inverse Jacovian Matrix
    J.Inv.pPos = [J.Inv.I J.Inv.O       J.Inv.O;
                  J.Inv.I J.Inv.pPar{1} J.Inv.pPar{2};
                  J.Inv.I J.Inv.pPar{3} J.Inv.pPar{4}];
        
    % Formation velocity
    trif.pPos.dQr = trif.pPos.dQd + trif.pPar.K1*tanh(trif.pPar.K2*trif.pPos.Qtil);
    
    % Robots velocities
    trif.pPos.dXr = J.Inv.pPos*trif.pPos.dQr;
end