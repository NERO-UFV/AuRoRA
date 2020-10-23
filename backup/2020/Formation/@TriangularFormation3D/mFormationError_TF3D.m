function mFormationError_TF3D(obj)

% Formation Error
obj.pPos.Qtil = obj.pPos.Qd - obj.pPos.Q;

% Quadrant adjust

% Beta
if abs(obj.pPos.Qtil(6)) > pi
    if obj.pPos.Qtil(6) > 0
        obj.pPos.Qtil(6) = -2*pi + obj.pPos.Qtil(6);
    else
        obj.pPos.Qtil(6) =  2*pi + obj.pPos.Qtil(6);
    end
end

% Phi
while abs(obj.pPos.Qtil(7)) > pi/2
    if obj.pPos.Qtil(7) > 0
        obj.pPos.Qtil(7) = -pi + obj.pPos.Qtil(7);
    else
        obj.pPos.Qtil(7) =  pi + obj.pPos.Qtil(7);
    end
end

% Theta
while abs(obj.pPos.Qtil(8)) > pi/2
    if obj.pPos.Qtil(8) > 0
        obj.pPos.Qtil(8) = -pi + obj.pPos.Qtil(8);
    else
        obj.pPos.Qtil(8) =  pi + obj.pPos.Qtil(8);
    end
end

% Psi
while abs(obj.pPos.Qtil(9)) > pi
    if obj.pPos.Qtil(9) > 0
        obj.pPos.Qtil(9) = -2*pi + obj.pPos.Qtil(9);
    else
        obj.pPos.Qtil(9) =  2*pi + obj.pPos.Qtil(9);
    end
end
