% Limpeza de variaveís e códigos
clear all
close all
clc

try
    fclose(instrfindall);
catch
end

%% Load Class
try
    % Load Classes
    RI = RosInterface;
    setenv('ROS_IP','192.168.0.158')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    
% Create OptiTrack object and initialize
%     OPT = OptiTrack;
%     OPT.Initialize;
    
        
    P=RPioneer(1,'P2');    
    
    % Joystick
    J = JoyControl;
    Joy = 1;
   
    disp('################### Load Class Success #######################');
    
catch ME
    disp(' ');
    disp(' ################### Load Class Issues #######################');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    
    rosshutdown;
    return;
    
end

%% Roda daqui pra baixo, aperta Ctrl + Enter para rodar essa parte amarela


% Definindo os parametros do pioneer
P.pPar.a = 0;
P.pPar.alpha = 0;
pgains = [.1 .1 1];

% Trajetoria
V_MAX = 0.75;

rx = 1;
ry = rx;

T_MAX = 2*pi*rx/V_MAX;
T_MAX = 30;

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
P.mCADplot(1,'k')
grid on
hold on
axis equal
axis([-1.5 1.5 -1.5 1.5])

% Armazenador de dados
DADOS = [];

% Temporizadores
T_AMOSTRAGEM = 1/30;
% T_PLOT=1/30 %PARA SIMULAÇÃO
T_PLOT = T_MAX+1; %PARA EXECUTAR NO PIONEER

T = tic;
Ta = tic;
Tp = tic;
T_dXd = tic;
% P.rEnableMotors; %Ativa motor do pioneer


% Laço de simulação
while toc(T) < T_MAX
% while true
    % Laço de controle
    if toc(Ta) > T_AMOSTRAGEM
        Ta = tic;
        
        XA = P.pPos.X;
        
        % Pegar sinais de posição do pioneer
        P.rGetSensorData;
        disp(P.pPos.X([1:3 6]))
        
        % Trajetoria
        XdA = Xd;
        Xd = [rx*cos(W*toc(T));
              ry*sin(W*toc(T));
              0];
          
%        Xd = [rx; 1.885*toc(T)/T_MAX; 0];
          
        dXd = (Xd-XdA)/toc(T_dXd);
        T_dXd = tic;
        
        P.pPos.Xd(1:3) = Xd;
        P.pPos.Xd(7:9) = dXd;
        
        % Controle
        P = fKinematicControllerExtended(P,pgains);
        
        if P.pSC.Ud(1) > .15
            P.pSC.Ud(1)= .15;
        end
        if P.pSC.Ud(2) > 2.09
            P.pSC.Ud(2)= 2.09;
        end
        % Armazenando dados
        DADOS(end+1,:) = [P.pPos.Xd' P.pPos.X' P.pSC.Ud' toc(T)];
        
        %                 1--12         13--24          25--26          27
        %                 P.pPos.Xd'    P.pPos.X'       P.pSC.Ud        toc(T)
        
        P = J.mControl(P);
        
        % Enviar sinais de controle
        P.rCommand;
       
        if J.pFlag == 1
            break 
        end
    
    end
    % Laço de plot
    if toc(Tp) > T_PLOT
        try
            P.mCADdel;
        catch
        end
        
        P.mCADplot(1,'k');
        Traj = plot([XdA(1) Xd(1)],[XdA(2) Xd(2)],'--k','LineWidth',1.6);
        Rastro = plot([XA(1) P.pPos.X(1)],[XA(2) P.pPos.X(2)],'r','LineWidth',1.6);
        drawnow
    end
end

P.pSC.Ud = [1; 0];
P.rCommand;

%P.rDisableMotors;
% gráficos

%% Graficos

% figure(2)
% plot(DADOS(:,end),DADOS(:,25),'b')
% hold on
% grid on
% plot(DADOS(:,end),DADOS(:,26),'r')
% plot([0 DADOS(end,end)],[mean(DADOS(:, 25)) mean(DADOS(:, 25))], 'k')
% plot([0 DADOS(end,end)],[mean(DADOS(:, 26)) mean(DADOS(:, 26))], 'c')
% legend('Vel. Linear', 'Vel. angular', 'Vel. Linear média','Vel. angular média')
% legend('Vel. Linear', 'Vel. angular')
% title('Sinais de controle')
% disp('linear:')
% disp(mean(DADOS(:,25)))
% disp('Angular:')
% disp(mean(DADOS(:,26)))
% 
% figure(3)
% plot(DADOS(:,end),(DADOS(:,1)-DADOS(:,13))*.2,'b')
% hold on
% grid on
% plot(DADOS(:,end),(DADOS(:,2)-DADOS(:,14))*.2,'r')
% legend('Vel. Linear', 'Vel. angular', 'Vel. Linear média','Vel. angular média')
% title('Sinais de controle')
% 
% 
% figure(4)
% plot(DADOS(:,end),DADOS(:,7),'b')
% hold on
% grid on
% plot(DADOS(:,end),DADOS(:,8),'r')
% plot(DADOS(:,end),sqrt(DADOS(:,7).^2+DADOS(:,8).^2))
% legend('')
% title('')
% 
% 
% figure(5)
% plot(DADOS(:,end),DADOS(:,13),'g')
% hold on
% grid on
% plot(DADOS(:,end),DADOS(:,14),'')
% legend('Vel. Linear', 'Vel. angular', 'Vel. Linear média','Vel. angular média')
% title('Sinais de controle')
% 
% 
