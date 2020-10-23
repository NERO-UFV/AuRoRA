clear all
close all
clc

% Carregando a classe do pioneer
P = Pioneer3DX;

% Definindo os parametros do pioneer
P.pPar.a = 0;
P.pPar.alpha = 0;
pgains = 10*[.2 .2 1];

% Trajetoria
V_MAX = 0.5;

rx = 1;
ry = rx;

T_MAX = 2*pi*rx/V_MAX;

W = 2*pi/T_MAX;

Xd = [rx*cos(W*0);
      ry*sin(W*0);
      0];
  
dXd = [-W*rx*sin(W*0);
       W*ry*cos(W*0);
       0];
   
P.pPos.Xd([1:3 6]) = [Xd; atan2(dXd(2),dXd(1))];
  
% dXd = (Xd-XdA)/toc(T_dXd);
% T_dXd = tic;

% Definir posição inicial do pioneer
P.rSetPose(P.pPos.Xd([1:3 6]));

% Plot
figure
subplot(2,2,[1 3])
P.mCADplot(1,'k')
grid on
hold on
axis equal
axis([-1.5 1.5 -1.5 1.5])


% Armazenador de dados
DADOS = [];

% Temporizadores
T_AMOSTRAGEM = 1/30;
T_PLOT = 1/30;

T = tic;
Ta = tic;
Tp = tic;
T_dXd = tic;

% Laço de simulação
while toc(T) < T_MAX
    % Laço de controle
    if toc(Ta) > T_AMOSTRAGEM
        Ta = tic;
        
        XA = P.pPos.X;
        % Pegar sinais de posição do pioneer
        P.rGetSensorData;
        
        % Trajetoria
        XdA = Xd;
        Xd = [rx*cos(W*toc(T));
              ry*sin(W*toc(T));
              0];
          
%         Xd = [rx; 1.885*toc(T)/T_MAX; 0];
          
        dXd = (Xd-XdA)/toc(T_dXd);
        T_dXd = tic;
        
        P.pPos.Xd(1:3) = Xd;
        P.pPos.Xd(7:9) = dXd;
        
        % Controle
        P = fKinematicControllerExtended(P,pgains);
        
        % Armazenando dados
        DADOS(end+1,:) = [P.pPos.Xd' P.pPos.X' P.pSC.Ud' toc(T)];
        
        %                 1--12         13--24          25--26          27
        %                 P.pPos.Xd'    P.pPos.X'       P.pSC.Ud        toc(T)
        
        % Enviar sinais de controle
        P.rSendControlSignals;
    end
    % Laço de plot
    if toc(Tp) > T_PLOT
        subplot(2,2,[1 3])
        try
            P.mCADdel;
        catch
        end
       
        P.mCADplot(1,'k');
        Traj = plot([XdA(1) Xd(1)],[XdA(2) Xd(2)],'--k','LineWidth',1.6);
        Rastro = plot([XA(1) P.pPos.X(1)],[XA(2) P.pPos.X(2)],'r','LineWidth',1.6);
        title('Simulação')
        xlabel('X [m]')
        ylabel('Y [m]')
        
        subplot(222)
        title('Sinal de controle no tempo')
        grid on
        
        subplot(224)
        title('Velocidades no tempo')
        grid on
        
        drawnow
    end
end

%% Gráficos
% figure(2)
subplot(222)
plot(DADOS(:,end),DADOS(:,25),'b','LineWidth',1.2)
hold on
plot(DADOS(:,end),DADOS(:,26),'r','LineWidth',1.2)
title('Sinal de controle no tempo')
xlabel('Tempo [s]')
legend('U_d','W_d')
grid on

% figure(3)
% plot(DADOS(:,end),(DADOS(:,1)-DADOS(:,13))*.2,'b')
% hold on
% grid on
% plot(DADOS(:,end),(DADOS(:,2)-DADOS(:,14))*.2,'r')

% figure(2)
subplot(224)
plot(DADOS(:,end),DADOS(:,7),'b','LineWidth',1.2)
hold on
plot(DADOS(:,end),DADOS(:,8),'r','LineWidth',1.2)
plot(DADOS(:,end),sqrt(DADOS(:,7).^2+DADOS(:,8).^2),'k','LineWidth',1.2)
plot(DADOS(:,end),(DADOS(:,7)+DADOS(:,8))/2,'g--','LineWidth',1.5)
title('Velocidades no tempo')
xlabel('Tempo [s]')
legend('dX_d','dY_d','U_d','Media (dX+dY)/2')
ylim([-1.2*max(sqrt(DADOS(:,7).^2+DADOS(:,8).^2)) ...
      +1.2*max(sqrt(DADOS(:,7).^2+DADOS(:,8).^2))])
grid on

