function sCalculateAngles(load,drone)
%% Load Disturbance

load.pPos.X(5) = asin((drone.pPos.X(1)-load.pPos.X(1))/load.pPar.l);
load.pPos.X(4) = asin((drone.pPos.X(2)-load.pPos.X(2))/load.pPar.l);


end