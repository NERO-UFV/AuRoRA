% Ilustrar a navega��o do ArDrone por atualiza��o da imagem

close all
clear
clc

% try
%     fclose(instrfindall);
    % Rotina para buscar pasta raiz
    PastaAtual = pwd;
    PastaRaiz = 'AuRoRA 2018';
    cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
    addpath(genpath(pwd))
% end

A = ArDrone;
A.mCADplot;

figure(1)
axis([-3 3 -3 3 0 3])
grid on
drawnow
hold on
pause(1)

% Constants:
g  = A.pPar.g;
m  = A.pPar.m;
mc = 0.005;
r  = 0.50;

% Posi��o P�ndulo
PX = A.pPos.X(1:3) + [0; 0; r];

Ix = A.pPar.Ixx;
Iy = A.pPar.Iyy;
Iz = A.pPar.Izz;
I  = 0.001;

% Refer�ncia 8 no plano
tmax = 60; % Tempo Simula��o em segundos
rx = 0;
ry = 0;
rz = 0;
w  = 2*pi/tmax;

X  = A.pPos.X(1:3)*ones(1,100);
Xd = [0 0 2]'*ones(1,100);

q   = zeros(8,1);
dq  = zeros(8,1);
qd  = zeros(8,1);
dqd = zeros(8,1);

TT = zeros(8,1);

% Active variables:
q(1) = A.pPos.X(1);
q(2) = A.pPos.X(2);
q(3) = A.pPos.X(3);

q(4) = A.pPos.X(4); % phi
q(5) = A.pPos.X(5); % theta
q(6) = A.pPos.X(6); % psi

q(7) = deg2rad(15); % alpha
q(8) = deg2rad(0); % beta

qa = q;

% =========================================================================
ta = 1/30;
tc = tic;
tp = tic;
t  = tic;

while toc(t) < tmax
    if toc(tc) > ta
        tc = tic;
        
        A.pPos.Xda = A.pPos.Xd;
        
        A.pPos.Xd(1) = rx*sin(w*toc(t));
        A.pPos.Xd(2) = ry*sin(2*w*toc(t));
        A.pPos.Xd(3) = 2+rz*sin(w*toc(t));
        
        %         A.pPos.Xd(7) = rx*w*cos(w*toc(t));
        %         A.pPos.Xd(8) = ry*2*w*cos(2*w*toc(t));
        %         A.pPos.Xd(9) = rz*w*cos(w*toc(t));
        
        A.pPos.Xd(7:9) = (A.pPos.Xd(1:3)-A.pPos.Xda(1:3))/ta;
        
        qd(1:3)  = A.pPos.Xd(1:3);
        dqd(1:3) = A.pPos.Xd(7:9);
        
        %%
        % Controlador de trajet�ria padr�o
        % A = cUnderActuatedController(A);
        
        
        %% Proposta controlador p�ndulo
        % Rotation matrix:
        qaa = qa;
        qa  = q;
        
        % Active variables:      
        q(1) = A.pPos.X(1);
        q(2) = A.pPos.X(2);
        q(3) = A.pPos.X(3);
        
        q(4) = A.pPos.X(4); % phi
        q(5) = A.pPos.X(5); % theta
        q(6) = A.pPos.X(6); % psi
        
        q(7) = q(7); % alpha
        q(8) = q(8); % beta
        
        dqa = dq;
        dq = (q-qa)/ta;
        
        
        % Analisar din�mica do sistema.
        % Resposta Natural sem a��o da gravidade
        
        % Inertia matrix:
        M = [
            m + mc,              0,                                                           0,              0,                                           0,                                                                             0,                                                                                                                                               mc*r*cos(q(7)),                      0;
            0,         m + mc,                                                           0,              0,                                           0,                                                                             0,                                                                                                                                                            0,         mc*r*cos(q(8));
            0,              0,                                                      m + mc,              0,                                           0,                                                                             0,                                                                                                  -(2^(1/2)*mc*r*cos(q(7)))/(cos(2*q(7)) + cos(2*q(8)))^(1/2),                      0;
            0,              0,                                                           0,             Ix,                                           0,                                                                -Ix*sin(q(5)),                                                                                                                                                            0,                      0;
            0,              0,                                                           0,              0,      Iy - Iy*sin(q(4))^2 + Iz*sin(q(4))^2,                                   -cos(q(4))*cos(q(5))*sin(q(4))*(Iy + Iz),                                                                                                                                                            0,                      0;
            0,              0,                                                           0, -Ix*sin(q(5)), -cos(q(4))*cos(q(5))*sin(q(4))*(Iy + Iz), Iz*cos(q(4))^2*cos(q(5))^2 + Iy*cos(q(5))^2*sin(q(4))^2 + Ix*sin(q(5))^2,                                                                                                                                                            0,                      0;
            mc*r*cos(q(7)),              0, -(2^(1/2)*mc*r*cos(q(7)))/(cos(2*q(7)) + cos(2*q(8)))^(1/2),              0,                                           0,                                                                             0, (I*sin(q(7))^2 - I + I*sin(q(8))^2 - 2*mc*r^2*cos(q(7))^2 + mc*r^2*cos(q(7))^2*sin(q(7))^2 + mc*r^2*cos(q(7))^2*sin(q(8))^2)/(sin(q(7))^2 + sin(q(8))^2 - 1),                      0;
            0, mc*r*cos(q(8)),                                                           0,              0,                                           0,                                                                             0,                                                                                                                                                            0, I + mc*r^2*cos(q(8))^2];
        
        
        % Coriolis matrix:
        C = [0, 0,                                                                                 0,                                                                                                                                    0,                                                                                                                                                                                                               0,                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                            -mc*r*sin(q(7))*dq(7),                                                                                                                                                                   0;
            0, 0,                                                                                 0,                                                                                                                                    0,                                                                                                                                                                                                               0,                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                                                0,                                                                                                                                               -mc*r*sin(q(8))*dq(8);
            0, 0,                                                                                 0,                                                                                                                                    0,                                                                                                                                                                                                               0,                                                                                                                                                                                                                                      0,                                                                                                  -(2*2^(1/2)*mc*r*sin(q(7))*dq(7) - 2*2^(1/2)*mc*r*sin(q(7))*cos(2*q(8))*dq(7) + 2^(1/2)*mc*r*cos(q(7))*sin(2*q(8))*dq(8))/(2*(cos(2*q(7)) + cos(2*q(8)))^(3/2)),                                                                                   -(2^(1/2)*mc*r*cos(q(7))*sin(2*q(8))*dq(7))/(2*(cos(2*q(7)) + cos(2*q(8)))^(3/2));
            0, 0,                                                                                 0,                                                                                                                                    0,                                                                                                              (cos(q(5))*dq(6)*(Iy*cos(2*q(4)) - Ix + Iz*cos(2*q(4))))/2 + sin(2*q(4))*dq(5)*(Iy/2 - Iz/2),                                                                                                   (cos(q(5))*dq(5)*(Iy*(2*cos(q(4))^2 - 1) - Ix + Iz*(2*cos(q(4))^2 - 1)))/2 - cos(q(4))*cos(q(5))^2*sin(q(4))*dq(6)*(Iy - Iz),                                                                                                                                                                                                                                                                0,                                                                                                                                                                   0;
            0, 0,                                                                                 0,                                 - (cos(q(5))*dq(6)*(Iy*cos(2*q(4)) - Ix + Iz*cos(2*q(4))))/2 - (sin(2*q(4))*dq(5)*(Iy - Iz))/2,                                                                                                                                                                              -(sin(2*q(4))*dq(4)*(Iy - Iz))/2, (cos(q(5))*(Ix*dq(4) - Iy*dq(4) - Iz*dq(4) + 2*Iy*sin(q(4))^2*dq(4) + 2*Iz*sin(q(4))^2*dq(4) - 2*Ix*sin(q(5))*dq(6) + 2*Iz*sin(q(5))*dq(6) + 2*Iy*sin(q(4))^2*sin(q(5))*dq(6) - 2*Iz*sin(q(4))^2*sin(q(5))*dq(6)))/2,                                                                                                                                                                                                                                                                0,                                                                                                                                                                   0;
            0, 0,                                                                                 0, cos(q(4))*cos(q(5))^2*sin(q(4))*dq(6)*(Iy - Iz) - (cos(q(5))*dq(5)*(Ix + Iy*(2*cos(q(4))^2 - 1) + Iz*(2*cos(q(4))^2 - 1)))/2, cos(q(5))*sin(q(5))*dq(6)*(Ix - Iz - Iy*sin(q(4))^2 + Iz*sin(q(4))^2) - (cos(q(5))*dq(4)*(Ix + Iy*(2*cos(q(4))^2 - 1) + Iz*(2*cos(q(4))^2 - 1)))/2 + cos(q(4))*sin(q(4))*sin(q(5))*dq(5)*(Iy + Iz),                                                                                                       cos(q(5))*sin(q(5))*dq(5)*(Ix - Iz - Iy*sin(q(4))^2 + Iz*sin(q(4))^2) + cos(q(4))*cos(q(5))^2*sin(q(4))*dq(4)*(Iy - Iz),                                                                                                                                                                                                                                                                0,                                                                                                                                                                   0;
            0, 0, -(2^(1/2)*mc*r*cos(q(7))*sin(2*q(8))*dq(8))/(2*(cos(2*q(7)) + cos(2*q(8)))^(3/2)),                                                                                                                                    0,                                                                                                                                                                                                               0,                                                                                                                                                                                                                                      0, (mc*r^2*cos(q(7))*(2*cos(q(7))^2*sin(q(7))*dq(7) - cos(q(7))^4*sin(q(7))*dq(7) + cos(q(8))^2*sin(q(7))*dq(7) - cos(q(8))^4*sin(q(7))*dq(7) - 2*cos(q(7))^2*cos(q(8))^2*sin(q(7))*dq(7) + cos(q(7))*cos(q(8))*sin(q(8))*dq(8)))/(sin(q(7))^2 + sin(q(8))^2 - 1)^2, (mc*r^2*cos(q(7))^2*cos(q(8))*sin(q(8))*dq(7))/(sin(q(7))^2 + sin(q(8))^2 - 1)^2 - (2^(1/2)*mc*r*cos(q(7))*sin(2*q(8))*dq(3))/(2*(cos(2*q(7)) + cos(2*q(8)))^(3/2));
            0, 0,  (2^(1/2)*mc*r*cos(q(7))*sin(2*q(8))*dq(7))/(2*(cos(2*q(7)) + cos(2*q(8)))^(3/2)),                                                                                                                                    0,                                                                                                                                                                                                               0,                                                                                                                                                                                                                                      0,                                                                                              (2^(1/2)*mc*r*cos(q(7))*sin(2*q(8))*dq(3))/(2*(cos(2*q(7)) + cos(2*q(8)))^(3/2)) - (mc*r^2*cos(q(7))^2*cos(q(8))*sin(q(8))*dq(7))/(sin(q(7))^2 + sin(q(8))^2 - 1)^2,                                                                                                                                       -(mc*r^2*sin(2*q(8))*dq(8))/2];
        
        % Gravity force matrix:
        G = [0;
            0;
            g*(m + mc);
            0;
            0;
            0;
            -(2^(1/2)*g*mc*r*sin(2*q(7)))/(2*(cos(2*q(7)) + cos(2*q(8)))^(1/2));
            -(2^(1/2)*g*mc*r*sin(2*q(8)))/(2*(cos(2*q(7)) + cos(2*q(8)))^(1/2))];
        
        
        % Sinal de controle
        % Controlador PD+Torque
        eta = 4*diag([1 1 1 1 1 1 1 1])*(qd-q) + 0.1*diag([1 1 1 1 1 1 1 1])*(dqd-dq) + ...
            100*[0 0 0 -TT(8) -TT(7) 0 0 0]';
        
        TT = M*eta + C*dq + G;

        rad2deg(q(5))
        
        % Integra��o Num�rica     
        ddq = M\(TT - G - C*dq);
        dq = dq + ddq*ta;
        q  = q  + dq*ta;
        
        A.pPos.X(1:6)  = q(1:6);
        A.pPos.X(7:12) = dq(1:6);

        
        % Posi��o do p�ndulo
        PX = A.pPos.X(1:3) + [r*sin(q(7)); r*sin(q(8)); r*sqrt(1-sin(q(7))^2-sin(q(8))^2)];
        
        %% Enviar sinais
        
        
        A.rSendControlSignals;
        
        X  = [X(:,2:end) A.pPos.X(1:3)];
        Xd = [Xd(:,2:end) A.pPos.Xd(1:3)];
        
    end
    if toc(tp) > 0.05
        tp = tic;
        A.mCADplot;
        h(1) = plot3(X(1,:),X(2,:),X(3,:),'-k');
        h(2) = plot3(Xd(1,:),Xd(2,:),Xd(3,:),'-.r');
        h(3) = plot3([X(1,end) PX(1)],[X(2,end) PX(2)],[X(3,end) PX(3)],'-or');
        drawnow
        delete(h)
    end
end