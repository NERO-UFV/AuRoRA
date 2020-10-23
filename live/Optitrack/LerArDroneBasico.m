


% Inicialização
close all
clear
clc
% Search root directory
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

figure(1)
axis([-3 3 -3 3 0 3])
grid on

A = ArDrone;


% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

tc = tic;
while true
    if toc(tc) > .1
       tc = tic;
        %  Get current rigid body information from optitrack
        rb = OPT.RigidBody;
       
        
        A = getOptData(rb,A);
  
        
        A.mCADplot;
        drawnow
    end
end
