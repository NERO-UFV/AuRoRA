clear; close all; clc;
% Close all the open connections
try
    fclose(instrfindall);
catch
end

%% Look for root directory
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

P = Pioneer3DX;

P.rConnect

%% Definir posição inicial

Xo = [0 0 0 0]

P.rSetPose(Xo);


%% Parte Gráfica
t=tic(t)
T = tic;
Tmax = 15;
figure();
subplot(211);
p1 = plot(T, P.pSC.Ud(1));
hold on
grid on
axis ([0 Tmax -2 2])
p2 = plot(T , P.pSC.U(2));
xlabel('XXXXX');
ylabel('YYYYY');
lg1 = legend('u_{d}','u');


t_inc=tic();
while toc() < Tmax

    if toc(t_inc) > Tinc:
        t_inc = tic();
        
    P.GetSensorData;
    






