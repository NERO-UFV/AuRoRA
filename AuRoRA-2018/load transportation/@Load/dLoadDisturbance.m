function Disturbance = dLoadDisturbance(load)

mDa = -load.pPar.m*load.pPar.l*cos(load.pPos.X(5))*load.pPos.dX(11) + load.pPar.m*load.pPar.l*sin(load.pPos.X(5))*load.pPos.X(11)^2;
mDb = -load.pPar.m*load.pPar.l*cos(load.pPos.X(4))*load.pPos.dX(10) + load.pPar.m*load.pPar.l*sin(load.pPos.X(4))*load.pPos.X(10)^2;
mDc = +load.pPar.m*load.pPar.g + load.pPar.m*load.pPar.l*(sin(load.pPos.X(5))*load.pPos.dX(11) + cos(load.pPos.X(5))*load.pPos.X(11)^2)+ load.pPar.m*load.pPar.l*(sin(load.pPos.X(4))*load.pPos.dX(10) + cos(load.pPos.X(4))*load.pPos.X(10)^2);


if mDa > load.pPar.m*load.pPar.g*2
    mDa = load.pPar.m*load.pPar.g*2;
end

if mDb > load.pPar.m*load.pPar.g*2
    mDb = load.pPar.m*load.pPar.g*2;
end

if mDc > load.pPar.m*load.pPar.g*2
    mDc = load.pPar.m*load.pPar.g*2;
end


mD = [mDa;...
    mDb;...
    mDc;];

if load.pPos.X(3) <= 0
    Disturbance = [0;0;0];
else
    Disturbance = mD;
end

end 