% Inserindo modelo drone
close all
clearvars
clc

try
    fclose(instrfindall);
end

try
    % Rotina para buscar pasta raiz
    PastaAtual = pwd;
    PastaRaiz = 'AuRoRA 2018';
    cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
    addpath(genpath(pwd))
end

A = ArDrone;
figure(1)
% SP(1) = subplot(2,2,1);
view(0,0),hold on
A.mCADplot;
axis([-2.5 2.5 -2.5 2.5 0 3])
grid on

% SP(2) = subplot(2,2,2);
% view(0,0),hold on
% axis([-1.5 1.5 -1.5 1.5 0 3])
% grid on
% 
% SP(3) = subplot(2,2,3);
% view(90,0),hold on
% axis([-1.5 1.5 -1.5 1.5 0 3])
% grid on
% 
% SP(4) = subplot(2,2,4);
% view(90,90),hold on
% axis([-1.5 1.5 -1.5 1.5 0 3])
% grid on

% SP(1).OuterPosition = [0.1 0.1 0.5 0.8];
% SP(2).OuterPosition = [0.7 0.63 0.3 0.33];
% SP(3).OuterPosition = [0.7 0.33 0.3 0.33];
% SP(4).OuterPosition = [0.7 0.0 0.3 0.33];

%Linux:
% SP(1).OuterPosition = [0.1 0.1 0.5 0.8];
% SP(2).OuterPosition = [0.7 0.66 0.3 0.33];
% SP(3).OuterPosition = [0.687 0.33 0.3 0.33];
% SP(4).OuterPosition = [0.694 0.0 0.3 0.33];

%%

pause(1)

% Constants:
g  = A.pPar.g;
m  = A.pPar.m;
mc = 0.005;
r  = 0.50;

% Pendulum position
PX = A.pPos.X(1:3) + [0; 0; r];

Ix = A.pPar.Ixx;
Iy = A.pPar.Iyy;
Iz = A.pPar.Izz;
I  = 2/5*mc*(0.02)^2;


tmax = 15; % Simulation time in seconds

X  = A.pPos.X(1:3)*ones(1,100);
Xd = [0 0 1]'*ones(1,100);
QQ = [];

q   = zeros(8,1);
dq  = zeros(8,1);
qd  = zeros(8,1);
dqd = zeros(8,1);

Tau = zeros(8,1);

% Active variables:
q(1) = A.pPos.X(1);
q(2) = A.pPos.X(2);
q(3) = A.pPos.X(3);

q(4) = A.pPos.X(4); % phi
q(5) = A.pPos.X(5); % theta
q(6) = A.pPos.X(6); % psi

q(7) = deg2rad( 5); % alpha
q(8) = deg2rad(-5); % beta

qa = q;

% =========================================================================
% cgains = [1.5 2 1.5 2 5 2; 1 15 1 15 1 2.5];
cgains = [1.5 2 1.5 2 5 2; 1 15 1 15 1 2.5]*1.259;  %converge um pouco mais rápido


Ganhos.kx1 = cgains(1,1);
Ganhos.kx2 = cgains(1,2);
Ganhos.kx3 = sqrt(4*Ganhos.kx1);
Ganhos.kx4 = sqrt(4*Ganhos.kx1*Ganhos.kx2)/Ganhos.kx3;

Ganhos.ky1 = cgains(1,3);
Ganhos.ky2 = cgains(1,4);
Ganhos.ky3 = sqrt(4*Ganhos.ky1);
Ganhos.ky4 = sqrt(4*Ganhos.ky1*Ganhos.ky2)/Ganhos.ky3;

Ganhos.kz1 = cgains(1,5);
Ganhos.kz2 = cgains(1,6);
Ganhos.kz3 = sqrt(4*Ganhos.kz1);
Ganhos.kz4 = sqrt(4*Ganhos.kz1*Ganhos.kz2)/Ganhos.kz3;

% phi
Ganhos.kp1 = cgains(2,1);
Ganhos.kp2 = cgains(2,2);
Ganhos.kp3 = sqrt(4*Ganhos.kp1);
Ganhos.kp4 = sqrt(4*Ganhos.kp1*Ganhos.kp2)/Ganhos.kp3;
% theta
Ganhos.kt1 = cgains(2,3);
Ganhos.kt2 = cgains(2,4);
Ganhos.kt3 = sqrt(4*Ganhos.kt1);
Ganhos.kt4 = sqrt(4*Ganhos.kt1*Ganhos.kt2)/Ganhos.kt3;
%psi
Ganhos.ks1 = cgains(2,5);
Ganhos.ks2 = cgains(2,6);
Ganhos.ks3 = sqrt(4*Ganhos.ks1);
Ganhos.ks4 = sqrt(4*Ganhos.ks1*Ganhos.ks2)/Ganhos.ks3;

% =========================================================================
ta = 1/30;
tc = tic;
tp = tic;
t  = tic;

while toc(t) < tmax
    if toc(tc) > ta
        tc = tic;
        tt = toc(t);
        
        A.pPos.Xda = A.pPos.Xd;
        
        A.pPos.Xd(1) = 1;
        A.pPos.Xd(2) = 0;
        A.pPos.Xd(3) = 1.5;       
        
        A.pPos.Xd(7:9) = (A.pPos.Xd(1:3)-A.pPos.Xda(1:3))/ta;
        
        qd(1:3)  = A.pPos.Xd(1:3);
        dqd(1:3) = A.pPos.Xd(7:9);               
        
        %% The proposed controller
        % Rotation matrix:
        
        % Active variables:
        q(1:6) = A.pPos.X(1:6);      
        q(7) = q(7); % alpha
        q(8) = q(8); % beta              
        
        dq(1:6) = A.pPos.X(7:12);
        
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
        C =[0, 0,                                                                                 0,                                                                                                                                    0,                                                                                                                                                                                                               0,                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                            -mc*r*sin(q(7))*dq(7),                                                                                                                                                                   0;
            0, 0,                                                                                 0,                                                                                                                                    0,                                                                                                                                                                                                               0,                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                                                0,                                                                                                                                               -mc*r*sin(q(8))*dq(8);
            0, 0,                                                                                 0,                                                                                                                                    0,                                                                                                                                                                                                               0,                                                                                                                                                                                                                                      0,                                                                                                  -(2*2^(1/2)*mc*r*sin(q(7))*dq(7) - 2*2^(1/2)*mc*r*sin(q(7))*cos(2*q(8))*dq(7) + 2^(1/2)*mc*r*cos(q(7))*sin(2*q(8))*dq(8))/(2*(cos(2*q(7)) + cos(2*q(8)))^(3/2)),                                                                                   -(2^(1/2)*mc*r*cos(q(7))*sin(2*q(8))*dq(7))/(2*(cos(2*q(7)) + cos(2*q(8)))^(3/2));
            0, 0,                                                                                 0,                                                                                                                                    0,                                                                                                              (cos(q(5))*dq(6)*(Iy*cos(2*q(4)) - Ix + Iz*cos(2*q(4))))/2 + sin(2*q(4))*dq(5)*(Iy/2 - Iz/2),                                                                                                   (cos(q(5))*dq(5)*(Iy*(2*cos(q(4))^2 - 1) - Ix + Iz*(2*cos(q(4))^2 - 1)))/2 - cos(q(4))*cos(q(5))^2*sin(q(4))*dq(6)*(Iy - Iz),                                                                                                                                                                                                                                                                0,                                                                                                                                                                   0;
            0, 0,                                                                                 0,                                 - (cos(q(5))*dq(6)*(Iy*cos(2*q(4)) - Ix + Iz*cos(2*q(4))))/2 - (sin(2*q(4))*dq(5)*(Iy - Iz))/2,                                                                                                                                                                              -(sin(2*q(4))*dq(4)*(Iy - Iz))/2, (cos(q(5))*(Ix*dq(4) - Iy*dq(4) - Iz*dq(4) + 2*Iy*sin(q(4))^2*dq(4) + 2*Iz*sin(q(4))^2*dq(4) - 2*Ix*sin(q(5))*dq(6) + 2*Iz*sin(q(5))*dq(6) + 2*Iy*sin(q(4))^2*sin(q(5))*dq(6) - 2*Iz*sin(q(4))^2*sin(q(5))*dq(6)))/2,                                                                                                                                                                                                                                                                0,                                                                                                                                                                   0;
            0, 0,                                                                                 0, cos(q(4))*cos(q(5))^2*sin(q(4))*dq(6)*(Iy - Iz) - (cos(q(5))*dq(5)*(Ix + Iy*(2*cos(q(4))^2 - 1) + Iz*(2*cos(q(4))^2 - 1)))/2, cos(q(5))*sin(q(5))*dq(6)*(Ix - Iz - Iy*sin(q(4))^2 + Iz*sin(q(4))^2) - (cos(q(5))*dq(4)*(Ix + Iy*(2*cos(q(4))^2 - 1) + Iz*(2*cos(q(4))^2 - 1)))/2 + cos(q(4))*sin(q(4))*sin(q(5))*dq(5)*(Iy + Iz),                                                                                                       cos(q(5))*sin(q(5))*dq(5)*(Ix - Iz - Iy*sin(q(4))^2 + Iz*sin(q(4))^2) + cos(q(4))*cos(q(5))^2*sin(q(4))*dq(4)*(Iy - Iz),                                                                                                                                                                                                                                                                0,                                                                                                                                                                   0;
            0, 0, -(2^(1/2)*mc*r*cos(q(7))*sin(2*q(8))*dq(8))/(2*(cos(2*q(7)) + cos(2*q(8)))^(3/2)),                                                                                                                                    0,                                                                                                                                                                                                               0,                                                                                                                                                                                                                                      0, (mc*r^2*cos(q(7))*(2*cos(q(7))^2*sin(q(7))*dq(7) - cos(q(7))^4*sin(q(7))*dq(7) + cos(q(8))^2*sin(q(7))*dq(7) - cos(q(8))^4*sin(q(7))*dq(7) - 2*cos(q(7))^2*cos(q(8))^2*sin(q(7))*dq(7) + cos(q(7))*cos(q(8))*sin(q(8))*dq(8)))/(sin(q(7))^2 + sin(q(8))^2 - 1)^2, (mc*r^2*cos(q(7))^2*cos(q(8))*sin(q(8))*dq(7))/(sin(q(7))^2 + sin(q(8))^2 - 1)^2 - (2^(1/2)*mc*r*cos(q(7))*sin(2*q(8))*dq(3))/(2*(cos(2*q(7)) + cos(2*q(8)))^(3/2));
            0, 0,  (2^(1/2)*mc*r*cos(q(7))*sin(2*q(8))*dq(7))/(2*(cos(2*q(7)) + cos(2*q(8)))^(3/2)),                                                                                                                                    0,                                                                                                                                                                                                               0,                                                                                                                                                                                                                                      0,                                                                                              (2^(1/2)*mc*r*cos(q(7))*sin(2*q(8))*dq(3))/(2*(cos(2*q(7)) + cos(2*q(8)))^(3/2)) - (mc*r^2*cos(q(7))^2*cos(q(8))*sin(q(8))*dq(7))/(sin(q(7))^2 + sin(q(8))^2 - 1)^2,                                                                                                                                       -(mc*r^2*sin(2*q(8))*dq(8))/2];
        
        % Gravity force matrix:
        G =[0;
            0;
            g*(m + mc);
            0;
            0;
            0;
            -(2^(1/2)*g*mc*r*sin(2*q(7)))/(2*(cos(2*q(7)) + cos(2*q(8)))^(1/2));
            -(2^(1/2)*g*mc*r*sin(2*q(8)))/(2*(cos(2*q(7)) + cos(2*q(8)))^(1/2))];
        
        A.pPos.Xtil = A.pPos.Xd - A.pPos.X;
        
        etax = A.pPos.dXd(7) + Ganhos.kx1*tanh(Ganhos.kx2*A.pPos.Xtil(1)) + Ganhos.kx3*tanh(Ganhos.kx4*A.pPos.Xtil(7));
        etay = A.pPos.dXd(8) + Ganhos.ky1*tanh(Ganhos.ky2*A.pPos.Xtil(2)) + Ganhos.ky3*tanh(Ganhos.ky4*A.pPos.Xtil(8));
        etaz = A.pPos.dXd(9) + Ganhos.kz1*tanh(Ganhos.kz2*A.pPos.Xtil(3)) + Ganhos.kz3*tanh(Ganhos.kz4*A.pPos.Xtil(9));
        
        etap = A.pPos.dXd(10) + Ganhos.kp1*tanh(Ganhos.kp2*A.pPos.Xtil(4)) + Ganhos.kp3*tanh(Ganhos.kp4*A.pPos.Xtil(10));
        etat = A.pPos.dXd(11) + Ganhos.kt1*tanh(Ganhos.kt2*A.pPos.Xtil(5)) + Ganhos.kt3*tanh(Ganhos.kt4*A.pPos.Xtil(11));
        etas = A.pPos.dXd(12) + Ganhos.ks1*tanh(Ganhos.ks2*A.pPos.Xtil(6)) + Ganhos.ks3*tanh(Ganhos.ks4*A.pPos.Xtil(12));

        etaa = 2*Ganhos.kt1*tanh(Ganhos.kt2*(qd(7)-q(7))) + Ganhos.kt3*tanh(Ganhos.kt4*(dqd(7)-dq(7)));
        etab = 2*Ganhos.kp1*tanh(Ganhos.kp2*(qd(8)-q(8))) + Ganhos.kp3*tanh(Ganhos.kp4*(dqd(8)-dq(8)));
        
        % eta = [etax; etay; etaz; etap; etat; etas; etaa; etab];

        
        % Referencia de Rolagem e Arfagem (Inserir Filtragem)       
        qda = qd; 
           
        qd(4) =  (q(7)*sin(A.pPos.X(6))-q(8)*cos(A.pPos.X(6)))*cos(A.pPos.X(5)) + ...
            atan2((etax*sin(A.pPos.X(6))-etay*cos(A.pPos.X(6)))*cos(A.pPos.X(5)),(etaz+A.pPar.g));
        qd(5) =  (q(7)*cos(A.pPos.X(6))+q(8)*sin(A.pPos.X(6))) + ...
            atan2((etax*cos(A.pPos.X(6))+etay*sin(A.pPos.X(6))),(etaz+A.pPar.g));
        
        dqd = (qd-q)/ta;
        
        % Sinal de controle
        eta = 0.15*diag([3 5 5 20 20 10 10 10])*tanh(qd-q) + 0.3*diag([1 1 1 15 15 10 5 5])*tanh(dqd-dq); % + ...
        
        Tau = M*eta + C*dq + G;
        
        % Rotation matrix:
        RotX = [1 0 0; 0 cos(q(4)) -sin(q(4)); 0 sin(q(4)) cos(q(4))];
        RotY = [cos(q(5)) 0 sin(q(5)); 0 1 0; -sin(q(5)) 0 cos(q(5))];
        RotZ = [cos(q(6)) -sin(q(6)) 0; sin(q(6)) cos(q(6)) 0; 0 0 1];
        
        R = RotZ*RotY*RotX;
        
        % Model-coupling matrices:
        mAt = R*[0 0 0 0; 0 0 0 0; 1 1 1 1];
        Fxyz = pinv(mAt)*Tau(1:3);
        
        mAr = [A.pPar.k1  A.pPar.k1 -A.pPar.k1  -A.pPar.k1; ...
            -A.pPar.k1  A.pPar.k1  A.pPar.k1  -A.pPar.k1; ...
            A.pPar.k2 -A.pPar.k2  A.pPar.k2  -A.pPar.k2];
        Fptp = pinv(mAr)*Tau(4:6);
        
        % torques in alpha and beta must be the contrary of torques in phi and theta
        mAp = [mAr(2,:);mAr(1,:)];
        Fab = pinv(mAp)*Tau(7:8);
        
        mA = [mAt;mAr;mAp];
        mAs = pinv(mA); %(A'*A)\A'; % Matriz Pseudo-Inversa de A: A-sharp
        
        Fd = mAs*Tau;  
        %% Low-level UAV dynamic model
        % 1: Fr -> Wr
        Wda = A.pSC.Wd;
        A.pSC.Wd = sqrt(Fd/A.pPar.Cf);
        
        % 2: Wr -> V
        Vr = -A.pPar.Vo + A.pPar.Jm*A.pPar.R/A.pPar.Km*(A.pSC.Wd-Wda)/A.pPar.Ts + ...
            (A.pPar.Bm*A.pPar.R/A.pPar.Km + A.pPar.Kb)*A.pSC.Wd + ...
            A.pPar.Ct*A.pPar.R/A.pPar.Km/A.pPar.r*A.pSC.Wd.^2;
        
        % 3: V -> Xr
        A.pSC.Xr(4) = A.pPos.X(4) + 1/(A.pPar.kdp+A.pPar.kpp*A.pPar.Ts)*...
            (A.pPar.kdp*(A.pSC.Xr(4)-A.pPos.X(4)) + 1/4*A.pPar.Ts*([1 1 -1 -1]*Vr));
        
        A.pSC.Xr(5) = A.pPos.X(5) + 1/(A.pPar.kdt+A.pPar.kpt*A.pPar.Ts)*...
            (A.pPar.kdt*(A.pSC.Xr(5)-A.pPos.X(5)) + 1/4*A.pPar.Ts*([-1 1 1 -1]*Vr));
        
        A.pSC.Xr(9) = A.pPos.X(9) + 1/(A.pPar.kdz+A.pPar.kpz*A.pPar.Ts)*...
            (A.pPar.kdz*(A.pSC.Xr(9)-A.pPos.X(9)) + 1/4*A.pPar.Ts*([1 1 1 1]*Vr));
        
        A.pSC.Xr(12) = A.pPos.X(12) + 1/(A.pPar.kds+A.pPar.kps*A.pPar.Ts)*...
            (A.pPar.kds*(A.pSC.Xr(12)-A.pPos.X(12)) + 1/4*A.pPar.Ts*([1 -1 1 -1]*Vr));
        
        % 4: Xr -> U
        A.pSC.Ud(1) =  A.pSC.Xr(4) /A.pPar.uSat(1);   % Phi
        A.pSC.Ud(2) = -A.pSC.Xr(5) /A.pPar.uSat(2);   % Theta
        A.pSC.Ud(3) =  A.pSC.Xr(9) /A.pPar.uSat(3);   % dZ
        A.pSC.Ud(4) = -A.pSC.Xr(12)/A.pPar.uSat(4);   % dPsi
        
        A.pSC.Ud = tanh(A.pSC.Ud);
        
        % Simulation data
        QQ = [QQ; [A.pPos.Xd(1:6)' A.pPos.X(1:6)'  A.pSC.Ud'.*A.pPar.uSat' q(7:8)' tt]]; 
        
        A.rSendControlSignals;
        
        % Numerical integration        
        ddq = M\(Tau - G - C*dq);
        dq = dq + ddq*ta;
        q  = q  + dq*ta;
        
        % Pendulum position
        PX = A.pPos.X(1:3) + [r*sin(q(7)); r*sin(q(8)); r*sqrt(1-sin(q(7))^2-sin(q(8))^2)];
        
        % Rastro
        X  = [X(:,2:end) A.pPos.X(1:3)];
        Xd = [Xd(:,2:end) A.pPos.Xd(1:3)];              
    end
    
    if toc(tp) > 0.1
        tp = tic;
        
%         subplot(SP(1))
        A.mCADplot;
        h(1) = plot3(X(1,:),X(2,:),X(3,:),'-k');
        h(2) = plot3(Xd(1,:),Xd(2,:),Xd(3,:),'-.r');
        h(3) = plot3([X(1,end) PX(1)],[X(2,end) PX(2)],[X(3,end) PX(3)],'-or');
        
%         subplot(SP(2))
%         h2(1) = plot3(X(1,:),X(2,:),X(3,:),'-k');
%         h2(2) = plot3(Xd(1,:),Xd(2,:),Xd(3,:),'-.r');
%         h2(3) = plot3([X(1,end) PX(1)],[X(2,end) PX(2)],[X(3,end) PX(3)],'-or');
%         
%         subplot(SP(3))
%         h3(1) = plot3(X(1,:),X(2,:),X(3,:),'-k');
%         h3(2) = plot3(Xd(1,:),Xd(2,:),Xd(3,:),'-.r');
%         h3(3) = plot3([X(1,end) PX(1)],[X(2,end) PX(2)],[X(3,end) PX(3)],'-or');
%         
%         subplot(SP(4))
%         h4(1) = plot3(X(1,:),X(2,:),X(3,:),'-k');
%         h4(2) = plot3(Xd(1,:),Xd(2,:),Xd(3,:),'-.r');
%         h4(3) = plot3([X(1,end) PX(1)],[X(2,end) PX(2)],[X(3,end) PX(3)],'-or');
        
        drawnow
        delete(h)   
%         delete(h2)
%         delete(h3)
%         delete(h4)
        
    end
end

figure
% figure(2)
% subplot(3,1,1)
plot(QQ(:,end),QQ(:,7),'-k','LineWidth',1.2)
hold on
plot(QQ(:,end),QQ(:,1),'--r')
legend('Real','Goal','Interpreter','latex','FontSize',10)
% title('x vs t','Interpreter','latex')
ylabel('x [m]','FontSize',16)
xlabel('Time [s]','FontSize',16)

figure
% subplot(3,1,2)
plot(QQ(:,end),QQ(:,8),'-k','LineWidth',1.2)
hold on
plot(QQ(:,end),QQ(:,2),'--r')
legend('Real','Goal','Interpreter','latex','FontSize',10)
% title('y vs t','Interpreter','latex')
ylabel('y [m]','FontSize',16)
xlabel('Time [s]','FontSize',16)

figure
% subplot(3,1,3)
plot(QQ(:,end),QQ(:,9),'-k','LineWidth',1.2)
hold on
plot(QQ(:,end),QQ(:,3),'--r')
legend('Real','Goal','Interpreter','latex','FontSize',10)
% title('z vs t','Interpreter','latex')
ylabel('z [m]','FontSize',16)
xlabel('Time [s]','FontSize',16)

figure
% subplot(2,1,2)
plot(QQ(:,end),rad2deg(QQ(:,17)),'-k','LineWidth',1.2)
hold on
plot(QQ(:,end),rad2deg(QQ(:,18)),'-r','LineWidth',1.2)
legend('$$\alpha$$','$$\beta$$','Interpreter','latex','FontSize',10)
%title('$\beta$ vs t','Interpreter','latex')
% legend('$\alpha$','$\beta$','Interpreter','latex')
ylabel('Pendulum Variables [�]','Interpreter','latex','FontSize',16)
xlabel('Time [s]','Interpreter','latex','FontSize',16)