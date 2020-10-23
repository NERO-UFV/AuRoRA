PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


OPT = OptiTrack;
OPT.Initialize;

% Load Classes
RI = RosInterface;
RI.rConnect('192.168.0.166');
B = Bebop(1,'B');


rb = OPT.RigidBody;

B = getOptData(rb,B);

B.pPos.X