function fDirTrans(linef)

    x1 = linef.pPos.X(1,1);
    y1 = linef.pPos.X(2,1);
    z1 = linef.pPos.X(3,1);
    x2 = linef.pPos.X(4,1);
    y2 = linef.pPos.X(5,1);
    z2 = linef.pPos.X(6,1);
    
    linef.pPos.Q(1) = x1;                                               % xf
    linef.pPos.Q(2) = y1;                                               % yf
    linef.pPos.Q(3) = z1;                                               % zf
    linef.pPos.Q(4) = sqrt((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);    % r_f
    linef.pPos.Q(5) = atan2((y2 - y1),(x2 - x1));                       % alpha_f
    linef.pPos.Q(6) = atan2(sqrt((x1 - x2)^2 + (y1 - y2)^2),(z2 - z1)); % delta_f
end