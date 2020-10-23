function rGetSensorData(p3dx)

% Store past position
p3dx.pPos.Xa = p3dx.pPos.X;

if p3dx.pFlag.Connected    
    % --------------------------------------------------------------------
    % MobileSim or Real P3DX
    % Robot pose from ARIA
    p3dx.pPos.Xc(1) = arrobot_getx/1000;       % x
    p3dx.pPos.Xc(2) = arrobot_gety/1000;       % y
    p3dx.pPos.Xc(6) = arrobot_getth/180*pi;    % psi
    
    % Robot velocities
    p3dx.pSC.Ua   = p3dx.pSC.U; 
    p3dx.pSC.U(1) = arrobot_getvel/1000;       % linear       
    p3dx.pSC.U(2) = arrobot_getrotvel/180*pi;  % angular
    
    K1 = [ cos(p3dx.pPos.Xc(6)), 0; ...
        sin(p3dx.pPos.Xc(6)), 0];
    
    K2 = [ cos(p3dx.pPos.Xc(6)), -p3dx.pPar.a*sin(p3dx.pPos.Xc(6)); ...
        sin(p3dx.pPos.Xc(6)), +p3dx.pPar.a*cos(p3dx.pPos.Xc(6))];

    p3dx.pPos.Xc(7:8) = K1 * p3dx.pSC.U;
    p3dx.pPos.Xc(12)  = p3dx.pSC.U(2);
    
    % Pose of the Control point
    p3dx.pPos.X([1 2 3]) = p3dx.pPos.Xc([1 2 3]) + ...
        [p3dx.pPar.a*cos(p3dx.pPos.Xc(6)); p3dx.pPar.a*sin(p3dx.pPos.Xc(6)); 0];
    p3dx.pPos.X([4 5 6]) = p3dx.pPos.Xc([4 5 6]); 
    p3dx.pPos.X(7:8) = K2 * p3dx.pSC.U;
    p3dx.pPos.X(12) = p3dx.pSC.U(2);    
    
else
    % Simulation       
    % Robot center position
    p3dx.pPos.Xc([1 2 6]) = p3dx.pPos.X([1 2 6]) - [p3dx.pPar.a*cos(p3dx.pPos.X(6)); p3dx.pPar.a*sin(p3dx.pPos.X(6)); 0];   
end