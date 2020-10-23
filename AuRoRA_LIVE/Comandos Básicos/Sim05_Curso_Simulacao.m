clearvars
close all
clc

try
    fclose(instrfindall);
catch
end
%%

J = JoyControl;
P = Pioneer3DX; % Pìoneer3DX Experimento
P.pPar.a = 0;
P.pPar.alpha = 0;

%%
% Temporizadores
Ta = tic;
Tp = tic;

figure
grid on
hold on
axis([-2 2 -2 2])
axis equal
%%
% while J.pFlag ~= 0 && Condição
% Condição será o erro de posição do robô (Xd - X)
d= 1 ;
P = parabaixo(P,d);


%%






