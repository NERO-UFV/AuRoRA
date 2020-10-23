function fFormationError(linef)

% Formation Error
linef.pPos.Qtil = linef.pPos.Qd - linef.pPos.Q;

% Quadrant adjust
if abs(linef.pPos.Qtil(5)) > pi
    if linef.pPos.Qtil(5) > 0
        linef.pPos.Qtil(5) = -2*pi + linef.pPos.Qtil(5);
    else
        linef.pPos.Qtil(5) = 2*pi + linef.pPos.Qtil(5);
    end
end

if abs(linef.pPos.Qtil(6)) > pi
    if linef.pPos.Qtil(6) > 0
        linef.pPos.Qtil(6) = -2*pi + linef.pPos.Qtil(6);
    else
        linef.pPos.Qtil(6) = 2*pi + linef.pPos.Qtil(6);
    end
end