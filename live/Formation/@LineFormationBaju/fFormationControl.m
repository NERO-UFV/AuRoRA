function fFormationControl(linef)
    % Direct transformation
    linef.fDirTrans;        % Get formation variables
    
    % Formation pose error
    linef.fFormationError;
    
    % Inverse Jacobian Matrix parameters {Formation vector: [xf yf zf rf alphaf deltaf]}
    rf = linef.pPos.Q(4);
    alphaf = linef.pPos.Q(5);
    deltaf = linef.pPos.Q(6);
    
    I = [1 0 0;
         0 1 0;
         0 0 1];
    O = [0 0 0;
         0 0 0;
         0 0 0];
    j{1} = [cos(alphaf)*sin(deltaf) -rf*sin(alphaf)*sin(deltaf) rf*cos(alphaf)*cos(deltaf);
            sin(alphaf)*sin(deltaf) rf*cos(alphaf)*sin(deltaf)  rf*sin(alphaf)*cos(deltaf);
            cos(deltaf)             0                           -rf*sin(deltaf)];
    
    % Inverse Jacobian Matrix
    Jinv = [I  O;
            I j{1}];
    
    % Formation velocity
    linef.pPos.dQr = linef.pPos.dQd + linef.pPar.K1*tanh(linef.pPar.K2*linef.pPos.Qtil);
    
    % Robots velocities
    linef.pPos.dXr = Jinv*linef.pPos.dQr;
end