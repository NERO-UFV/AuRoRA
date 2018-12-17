% Guiar drone virtual usando joystick
% Testar modelo dinâmico

close all
clear
clc

try
    fclose(instrfindall);
end
% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

A = ArDrone;

% Conectar Joystick
J = JoyControl;

tmax = 60; % Tempo Simulação em segundos
X = zeros(1,19); % Dados correntes da simulação

figure(1)
axis([-3 3 -3 3 0 3])
grid on
drawnow
pause(1)

% =========================================================================
t = tic;
tc = tic;
tp = tic;

while toc(t) < tmax
    if toc(tc) > 1/30
        tc = tic;
        
        A.pSC.Ud = zeros(4,1);
        
        % Controlador Joystick
        A = J.mControl(A);
        
        % Obter dados de voo
        A.rSendControlSignals;
%         A.sDynamicModel
        
    end
    if toc(tp) > 0.05
        tp = tic;
        A.mCADplot;
        drawnow
    end
    
end
