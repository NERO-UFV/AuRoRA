function tDirTrans(trif)

    x = trif.pPos.X(1,1);
    y = trif.pPos.X(2,1);
    z = trif.pPos.X(3,1);
    x1 = trif.pPos.X(4,1);
    y1 = trif.pPos.X(5,1);
    z1 = trif.pPos.X(6,1);
    x2 = trif.pPos.X(7,1);
    y2 = trif.pPos.X(8,1);
    z2 = trif.pPos.X(9,1);
    
    pf = sqrt((x1 - x)^2 + (y1 - y)^2 + (z1 - z)^2);
    qf = sqrt((x2 - x)^2 + (y2 - y)^2 + (z2 - z)^2);
    sf = sqrt((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
    
    PA1 = [x1-x y1-y z1-z]';
    PA2 = [x2-x y2-y z2-z]';
    N = cross(PA2,PA1);
    
    phif = atan2(norm(N(1:2)),N(3)) - pi/2;
    psif = pi/2 + atan2(N(2),N(1));
    
    % Matrizes de rotação
    % Rotação em X
    Rx = [1 0         0;
          0 cos(phif) -sin(phif);
          0 sin(phif) cos(phif)];
    % Rotação em Z
    Rz = [cos(psif) -sin(psif) 0;
          sin(psif) cos(psif)  0;
          0         0          1];
    % Rotação em Y,X e Z nesta ordem
    PA1 = Rz\PA1;
    PA1 = Rx\PA1;    
    
    thetaf = atan2(PA1(1),PA1(3));
    
    trif.pPos.Q(1,1) = x;                                                % xf
    trif.pPos.Q(2,1) = y;                                                % yf
    trif.pPos.Q(3,1) = z;                                                % zf
    trif.pPos.Q(4,1) = thetaf;                                           % thetaf
    trif.pPos.Q(5,1) = phif;                                             % phif
    trif.pPos.Q(6,1) = psif;                                             % psif
    trif.pPos.Q(7,1) = pf;                                               % pf
    trif.pPos.Q(8,1) = qf;                                               % qf
    trif.pPos.Q(9,1) = acos((pf^2 + qf^2 - sf^2)/(2*pf*qf));             % betaf
end