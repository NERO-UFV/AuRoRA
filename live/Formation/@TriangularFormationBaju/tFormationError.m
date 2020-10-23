function tFormationError(trif)

% Formation Error
trif.pPos.Qtil = trif.pPos.Qd - trif.pPos.Q;

% Quadrant adjust
for ii = [4:6 9]
while abs(trif.pPos.Qtil(ii)) > pi
    if trif.pPos.Qtil(ii) > 0
        trif.pPos.Qtil(ii) = -2*pi + trif.pPos.Qtil(ii);
    else
        trif.pPos.Qtil(ii) = 2*pi + trif.pPos.Qtil(ii);
    end
end
end
