function tInvTrans(trif)

    xf     = trif.pPos.Qd(1);
    yf     = trif.pPos.Qd(2);
    zf     = trif.pPos.Qd(3);
    thetaf = trif.pPos.Qd(4);
    phif   = trif.pPos.Qd(5);
    psif   = trif.pPos.Qd(6);
    pf     = trif.pPos.Qd(7);
    qf     = trif.pPos.Qd(8);
    betaf  = trif.pPos.Qd(9);
    
    % Matrizes de rotação
    % Rotação em X
    Rx = [1 0         0;
          0 cos(phif) -sin(phif);
          0 sin(phif) cos(phif)];
    % Rotação em Y
    Ry = [cos(thetaf)  0 sin(thetaf);
          0            1 0;
          -sin(thetaf) 0 cos(thetaf)];
    % Rotação em Z
    Rz = [cos(psif) -sin(psif) 0;
          sin(psif) cos(psif)  0;
          0         0          1];
    % Rotação em Y,X e Z nesta ordem
    R = Rz*Rx*Ry;

    % Matriz XF, PF e QF, representando a posição do pioneer, drone 1 e drone 2
    % respectivamente antes de aplicar as rotações
    PF = [0; 0; pf];
    QF = [qf*sin(betaf); 0; qf*cos(betaf)];
    XF = [xf; yf; zf];

    % Posição dos robos
    trif.pPos.Xd = [XF;
                    R*PF + XF;
                    R*QF + XF];
end