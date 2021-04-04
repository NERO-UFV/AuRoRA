clearvars
close all
clc

% Criando as classes
P = Pioneer3DX;
T = Pioneer3DX;
Encaixe = zeros(12,1);

P.pPar.alpha = 0;
P.pPar.a = 0;
T.pPar.alpha = 0;
T.pPar.a = 0;

% Definindo as posi��es iniciais
R = 0.295;
F = 0.325;

P.pPos.X(1:2) = [1; 0];
P.pPos.X(6) = pi/2;
T.pPos.X(6) = P.pPos.X(6);
Xr = P.pPos.X;
Encaixe(1:2) = [Xr(1) - R*cos(Xr(6));
                Xr(2) - R*sin(Xr(6))];
T.pPos.X(1:2) = [Encaixe(1) - F*cos(T.pPos.X(6));
                 Encaixe(2) - F*sin(T.pPos.X(6))];
             
P.rSetPose([Xr(1:3)' Xr(6)]);
T.rSetPose([T.pPos.X(1:3)' T.pPos.X(6)]);

% Plot do reboque e pioneer

figure(1)
P.mCADplot(1,'k');
hold on
grid on
% T.mCADplot(1,'r');
H(1) = plot(Encaixe(1),Encaixe(2),'xk','LineWidth',1.6,'MarkerSize',15);
H(2) = plot([Encaixe(1) P.pPos.X(1)],[Encaixe(2) P.pPos.X(2)],'-k','LineWidth',1.6);
H(3) = plot([Encaixe(1) T.pPos.X(1)],[Encaixe(2) T.pPos.X(2)],'-r','LineWidth',1.6);
[truck(1),truck(2)] = draw_rectangle([T.pPos.X(1),T.pPos.X(2)],.45,.35,T.pPos.X(6),[1,0,0]);
axis equal
axis([-1.5 1.5 -1.5 1.5])
pause

% Temporizadores

T_MAX = 30;
W = 2*pi/T_MAX;
rx = 1;
ry = rx;
T_AMOSTRAGEM = P.pPar.Ts;
T_PLOT = 0.1;
Xd = [rx*cos(0);
      ry*sin(0);
      pi/2];
XdA = Xd;
dXd = (Xd-XdA);
  
Ts = tic;
Ta = tic;
Tp = tic;
D = tic;
I = tic;

% Definindo velocidades

% P.pSC.Ud=[0.1, 0.1];

while toc(Ts) < T_MAX*2
    if toc(Ta) > T_AMOSTRAGEM
        Ta = tic;
        
        XdA = Xd;
        Xd = [rx*cos(W*toc(Ts));
              ry*sin(W*toc(Ts));
              atan2(dXd(2),dXd(1))];
        dXd = (Xd-XdA)/toc(D);
        D = tic;
        psi = P.pPos.X(6)-T.pPos.X(6);
        if abs(psi) > pi
            if psi > 0
                psi = -2*pi + psi;
            else
                psi = 2*pi + psi;
            end
        end
        dTheta = 1/F*(norm(dXd(1:2))*sin(psi)-R*W*cos(psi));
%         DD = norm(dXd(1:2))/dXd(3);
%         T.pPos.X(6) = P.pPos.X(6) - 2*atan2(F,DD);
        T.pPos.X(6) = T.pPos.X(6) + toc(I)*dTheta;
        if abs(T.pPos.X(6)) > pi
            if T.pPos.X(6) > 0
                T.pPos.X(6) = -2*pi + T.pPos.X(6);
            else
                T.pPos.X(6) = 2*pi + T.pPos.X(6);
            end
        end
        I = tic;
        
        disp(psi*180/pi)
        
        P.pPos.X([1 2 6]) = Xd;
        Xr = P.pPos.X;
        Encaixe(1:2) = [Xr(1) - R*cos(Xr(6));
                        Xr(2) - R*sin(Xr(6))];
        T.pPos.X(1:2) = [Encaixe(1) - F*cos(T.pPos.X(6));
                         Encaixe(2) - F*sin(T.pPos.X(6))];
        
        
    end
    if toc(Tp) > T_PLOT
        Tp = tic;
        
        try
            P.mCADdel;
            delete(truck)
            delete(H);
        catch
        end
        
        P.rSetPose([Xr(1:3)' Xr(6)]);
        T.rSetPose([T.pPos.X(1:3)' T.pPos.X(6)]);
        
        P.mCADplot(1,'k');
        H(1) = plot(Encaixe(1),Encaixe(2),'xk','LineWidth',1.6,'MarkerSize',15);
        H(2) = plot([Encaixe(1) P.pPos.X(1)],[Encaixe(2) P.pPos.X(2)],'-k','LineWidth',1.6);
        H(3) = plot([Encaixe(1) T.pPos.X(1)],[Encaixe(2) T.pPos.X(2)],'-r','LineWidth',1.6);
        [truck(1),truck(2)] = draw_rectangle([T.pPos.X(1),T.pPos.X(2)],.45,.35,T.pPos.X(6),[1,0,0]);
        axis([-1.5 1.5 -1.5 1.5])
        drawnow
    end
end

% Gr�fico das velocidades
plot(P.pPos.Xd,toc(Ts));
