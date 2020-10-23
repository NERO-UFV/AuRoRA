function mFormationError(obj)

% Formation Error
obj.pPos.Qtil = obj.pPos.Qd - obj.pPos.Q;

% Quadrant adjust
if abs(obj.pPos.Qtil(5)) > pi
    if obj.pPos.Qtil(5) > 0
        obj.pPos.Qtil(5) = -2*pi + obj.pPos.Qtil(5);
    else
        obj.pPos.Qtil(5) =  2*pi + obj.pPos.Qtil(5);
    end
end

if abs(obj.pPos.Qtil(6)) > pi
    if obj.pPos.Qtil(6) > 0
        obj.pPos.Qtil(6) = -2*pi + obj.pPos.Qtil(6);
    else
        obj.pPos.Qtil(6) =  2*pi + obj.pPos.Qtil(6);
    end
end
