function iControlVariables(pendulum)

% Platform + Pendulum's cartesian coordinates 
pendulum.pPos.Xd = [0 0 0 0 0 pendulum.pPar.r]';
pendulum.pPos.X  = [0 0 0 0 0 pendulum.pPar.r]';
