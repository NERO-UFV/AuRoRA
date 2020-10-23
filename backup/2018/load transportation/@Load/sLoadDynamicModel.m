function load = sLoadDynamicModel(load,drone)

%% Load Dynamic Model Update
drone.pPos.dX = (drone.pPos.X - drone.pPos.Xa)/drone.pPar.Ts;

load.pPos.dX(11) = 1/(load.pPar.m*load.pPar.l^2)*(-.05*load.pPos.X(11)+load.pPar.m*load.pPar.l*(...
    cos(load.pPos.X(5))*( drone.pPos.dX(7) - 2*load.pPos.X(11)*drone.pPos.X(9)) + ...
    sin(load.pPos.X(5))*(-drone.pPos.dX(9) - 2*load.pPos.X(11)*drone.pPos.X(7) - drone.pPar.g)));

load.pPos.X(11) = load.pPos.X(11) + load.pPos.dX(11)*drone.pPar.Ts;
load.pPos.X(5) = load.pPos.X(5) + load.pPos.X(11)*drone.pPar.Ts;

load.pPos.dX(10) = 1/(load.pPar.m*load.pPar.l^2)*(-.05*load.pPos.X(10)+load.pPar.m*load.pPar.l*(...
    cos(load.pPos.X(4))*( drone.pPos.dX(8) - 2*load.pPos.X(10)*drone.pPos.X(9)) + ...
    sin(load.pPos.X(4))*(-drone.pPos.dX(9) - 2*load.pPos.X(10)*drone.pPos.X(8) - drone.pPar.g)));

load.pPos.X(10) = load.pPos.X(10) + load.pPos.dX(10)*drone.pPar.Ts;
load.pPos.X(4) = load.pPos.X(4) + load.pPos.X(10)*drone.pPar.Ts;

% Postura da carga
load.pPos.X(1) = drone.pPos.X(1) - load.pPar.l*sin(load.pPos.X(5));
load.pPos.X(2) = drone.pPos.X(2) - load.pPar.l*sin(load.pPos.X(4));
load.pPos.X(3) = drone.pPos.X(3) - load.pPar.l*cos(load.pPos.X(4))*cos(load.pPos.X(5));

if load.pPos.X(3) < 0
    load.pPos.X(3) = 0;
end	


end