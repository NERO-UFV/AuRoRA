function sLoadDisturbance(load)

%% Load Disturbance
lDa = -load.pPar.m*load.pPar.l*cos(load.pPos.X(5))*load.pPos.dX(11) + load.pPar.m*load.pPar.l*sin(load.pPos.X(5))*load.pPos.X(11)^2;
lDb = -load.pPar.m*load.pPar.l*cos(load.pPos.X(4))*load.pPos.dX(10) + load.pPar.m*load.pPar.l*sin(load.pPos.X(4))*load.pPos.X(10)^2;
lDc = + load.pPar.m*obj.pPar.Corpo.g + load.pPar.m*load.pPar.l*(sin(load.pPos.X(5))*load.pPos.dX(11) + cos(load.pPos.X(5))*load.pPos.X(11)^2)+ load.pPar.m*load.pPar.l*(sin(load.pPos.X(4))*load.pPos.dX(10) + cos(load.pPos.X(4))*load.pPos.X(10)^2);

mD = [lDa;...
      lDb;...
      lDc];	

end