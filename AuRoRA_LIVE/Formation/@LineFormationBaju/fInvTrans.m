function fInvTrans(linef)

    xf     = linef.pPos.Qd(1);
    yf     = linef.pPos.Qd(2);
    zf     = linef.pPos.Qd(3);
    rf     = linef.pPos.Qd(4);
    alphaf = linef.pPos.Qd(5);
    deltaf = linef.pPos.Qd(6);

    linef.pPos.Xd(1,1) = xf;
    linef.pPos.Xd(2,1) = yf;
    linef.pPos.Xd(3,1) = zf;
    linef.pPos.Xd(4,1) = xf + rf*cos(alphaf)*sin(deltaf);
    linef.pPos.Xd(5,1) = yf + rf*sin(alphaf)*sin(deltaf);
    linef.pPos.Xd(6,1) = zf + rf*cos(deltaf);
end
