% Exibir imagem ArDrone conectado

close all
clear
clc
try
    fclose(instrfindall);
end
addpath(genpath(pwd))

figure(1)
axis([-1 1 -1 1 -0.5 0.5])
grid on
drawnow

A = ArDrone;
A.rConnect;

tmax = 30; % Tempo Simulação em segundos

% =========================================================================
t  = tic; % Tempo de simulação
tc = tic; % Tempo de controle
tp = tic; % Tempo de plotagem e exibição

%%
while toc(t) < tmax
    if toc(tc) > 1/30
        tc = tic;
        
        % Obter dados de voo
        A.rGetStatusRawData
        
        % Atribuir variáveis
        A.pPos.X(4) = A.pCom.cRawData(3)*pi/180;        
        A.pPos.X(5) = A.pCom.cRawData(2)*pi/180;        
        A.pPos.X(6) = A.pCom.cRawData(4)*pi/180;  
        
        A.pPos.X([4 5 6])
    end
    if toc(tp) > 0.05
        tp = tic;
        A.mCADplot;              
        drawnow
    end
end
%% 
A.rDisconnect