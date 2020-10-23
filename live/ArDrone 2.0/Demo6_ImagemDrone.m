% Ilustrar a navegação do ArDrone por atualização da imagem

close all
clear
clc
try
    fclose(instrfindall);
end
addpath(genpath(pwd))

A = ArDrone;
A.mCADplot;  

figure(1)
axis([-5 5 -5 5 0 4])
grid on
drawnow
pause(1)

tmax = 30; % Tempo Simulação em segundos

% Referência 8 no plano
rx = 4;
ry = 3;
rz = 1;
w  = 2*pi*0.1;

% =========================================================================
t  = tic;
tc = tic;
tp = tic;

while toc(t) < tmax
    if toc(tc) > 1/30
        tc = tic;
        
        A.pPos.X(1) = rx*sin(w*toc(t));        
        A.pPos.X(2) = ry*sin(2*w*toc(t));        
        A.pPos.X(3) = 2+rz*sin(w*toc(t));               
    end
    if toc(tp) > 0.05
        tp = tic;
        A.mCADplot;              
        drawnow
    end
end