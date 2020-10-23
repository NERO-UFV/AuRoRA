function rSetPose(p3dx,Xo)
if nargin > 1
    p3dx.pPos.Xc([1 2 3 6]) = Xo;
end

p3dx.pPos.X([1 2 3 6]) = p3dx.pPos.Xc([1 2 3 6]) + ...
    [cos(p3dx.pPos.X(6)) -sin(p3dx.pPos.X(6)) 0 0; sin(p3dx.pPos.X(6)) cos(p3dx.pPos.X(6)) 0 0; 0 0 1 0; 0 0 0 1]*...
    [p3dx.pPar.a*cos(p3dx.pPar.alpha); p3dx.pPar.a*sin(p3dx.pPar.alpha); 0; 0];

p3dx.pPos.Xa = p3dx.pPos.X;

if p3dx.pFlag.Connected
    arrobot_setpose(p3dx.pPos.Xc(1)*1000,p3dx.pPos.Xc(2)*1000,p3dx.pPos.Xc(6)*180/pi);
end
