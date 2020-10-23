function p3dx = fKinematicControllerExtended(p3dx,pgains)

% p3dx = fKinematicController(p3dx,pgains)
% pgains = [kx ky kpsi]

% Control gains
if nargin < 2
    pgains = [0.2 0.2 1];
end

% Computing the pose error
p3dx.pPos.Xtil = p3dx.pPos.Xd - p3dx.pPos.X;

% Extended Kinematic model
vx = p3dx.pPos.Xd(7) + pgains(1)*p3dx.pPos.Xtil(1);
vy = p3dx.pPos.Xd(8) + pgains(2)*p3dx.pPos.Xtil(2);

% For a*cos(alpha) ~= 0
if abs(p3dx.pPar.alpha) < pi/2 && p3dx.pPar.a > 0
    vw = (-sin(p3dx.pPos.X(6))*vx + cos(p3dx.pPos.X(6))*vy)/(p3dx.pPar.a*cos(p3dx.pPar.alpha));
else
    p3dx.pPos.Xd(6)  = atan2(vy,vx);
    p3dx.pPos.Xd(12) = (p3dx.pPos.Xd(6)-p3dx.pPos.Xda(6))/0.1;
    p3dx.pPos.Xtil(6) = p3dx.pPos.Xd(6) - p3dx.pPos.X(6);
    if abs(p3dx.pPos.Xtil(6)) > pi
        if p3dx.pPos.Xtil(6) > 0
            p3dx.pPos.Xtil(6) = - 2*pi + p3dx.pPos.Xd(6) - p3dx.pPos.X(6);
        else
            p3dx.pPos.Xtil(6) =   2*pi + p3dx.pPos.Xd(6) - p3dx.pPos.X(6);
        end
    end
    vw = 0*(p3dx.pPos.Xd(12)) + pgains(3)*p3dx.pPos.Xtil(6);
end

p3dx.pSC.Ud(2) = vw;
p3dx.pSC.Ud(1) = vx*cos(p3dx.pPos.X(6)) + vy*sin(p3dx.pPos.X(6)) + p3dx.pPar.a*sin(p3dx.pPar.alpha)*vw;

% Saturation of the control signal, based on the P3DX robot's datasheet
if abs(p3dx.pSC.Ur(1)) > 0.75
    p3dx.pSC.Ur(1) = sign(p3dx.pSC.Ur(1))*0.75;
end
if abs(p3dx.pSC.Ur(2)) > 1
    p3dx.pSC.Ur(2) = sign(p3dx.pSC.Ur(2))*1;
end
end