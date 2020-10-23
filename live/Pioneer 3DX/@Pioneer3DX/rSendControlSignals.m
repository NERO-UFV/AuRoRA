function rSendControlSignals(p3dx)

if p3dx.pFlag.Connected == 1
    % Experiment Mode: MobileSim or P3DX
    arrobot_setvel(p3dx.pSC.Ud(1)*1000);      % Linear velocity
    arrobot_setrotvel(p3dx.pSC.Ud(2)/pi*180); % Angular velocity
    
    % Stand-by mode
    p3dx.pSC.Ud = [0; 0];
else
    % Simulation Mode
    p3dx.pSC.U = p3dx.pSC.Ud;
    p3dx.sKinematicModel;
end


