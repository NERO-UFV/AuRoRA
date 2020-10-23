clear; close all; clc;
try
    fclose(instrfindall);
catch
end

%
% % Look for root folder
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))

%% Load Classes

%% Load Class
    % Load Classes
      % Create OptiTrack object and initialize
    OPT = OptiTrack;
    OPT.Initialize;
    
    % Initiate classes
    A{1} = ArDrone;
    A{2} = ArDrone;
    idA{1} = getID(OPT,ArDrone,1);
    idA{2} = getID(OPT,ArDrone,2);
    
    % Joystick
    J = JoyControl;
    

%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'land', ...
                         'Callback', 'btnEmergencia=1', ...
                         'Position', [50 50 400 300]);

%% Variable initialization
data = [];

% Time variables initialization
Xd = [0 0 0 0];
dXd = [0 0 0 0];

fprintf('\nStart..............\n\n');

A{1}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216  ]';
A{2}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216  ]';

% Bebop e Ardrone Takeoff
disp('Start Take Off Timming....');
A{1}.pPar.ip = '192.168.1.61';
A{1}.rConnect;


A{2}.pPar.LocalPortControl = 5558;
A{2}.pPar.LocalPortState = 5552;
A{2}.pPar.ip = '192.168.1.40';
A{2}.rConnect;

A{1}.rTakeOff;
A{2}.rTakeOff;

pause(5);
disp('Taking Off End Time....');

% timers
T_exp = 120;
T_run = 1/30;       % Tempo de amostragem do experimento
t_run = tic;
t_task = tic;
T_task = 13;
i_task = 0;
t_exp = tic;


t  = tic;

A{2}.pPar.ti = tic;
A{1}.pPar.ti = tic;
A{2}.pPar.Ts = 1/30;
A{1}.pPar.Ts = 1/30;

% Dados trajetória
rX = 1;           % [m]
rY = 1;           % [m]
T = 30;             % [s]
w = 2*pi/T;         % [rad/s]

    while toc(t) < T_exp
        
        if toc(t_run) > T_run
            
            t_run = tic;
            t_atual = toc(t);

            
%% TAREFA

%Trajetória   

%% Ciclo 1
if t_atual < T_exp

         Xd_1 = [0;
            0;
            .5;
            0];
   
        Xd_2 = Xd_1;    
        Xd_2(3) = Xd_1(3) + 1.5;      

        dXd_1 = [0;
                0;
                0;
                0];

        dXd_2 = dXd_1;   
        
%         Xd_1 = [rX*cos(w*t_atual);
%             rY*sin(w*t_atual);
%             1;
%             0];
%    
%         Xd_2 = Xd_1;    
%         Xd_2(3) = Xd_1(3) + 1.5;      
% 
%         dXd_1 = [-w*rX*sin(w*t_atual);
%                 w*rY*cos(w*t_atual);
%                 0;
%                 0];
% 
%         dXd_2 = dXd_1;

%% LEITURA DE POSIÇÃO E VELOCIDADE
       
% OPTITRACK
        rb = OPT.RigidBody;
        if rb(idA{1}).isTracked
            A{1} = getOptData(rb(idA{1}),A{1});
            A{1}.pPos.X
        end       
%         
        if rb(idA{2}).isTracked
            A{2} = getOptData(rb(idA{2}),A{2});
            A{2}.pPos.X
        end       


%% CONTROLE DINÂMICO DOS ROBÔS
% Atribuindo trajetória

        A{1}.pPos.Xd([1:3 6]) = Xd_2;
        A{2}.pPos.Xd([1:3 6]) = Xd_1;
        

        A{1}.pPos.Xd([7:9 12]) = dXd_2;
        A{2}.pPos.Xd([7:9 12]) = dXd_1;
        

% Controladores         

 
        
% ArDrone

        
          A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});
          A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
          
        A{2}.pPar.ti = tic;
        A{1}.pPar.ti = tic;
        
        
    if t_atual > 3/4*T
        A{2}.rLand;
    end  
  
% Atribuindo comandos
        A{1} = J.mControl(A{1});                    % joystick command (priority)
        A{1}.rSendControlSignals;
        A{2} = J.mControl(A{2});                    % joystick command (priority)
        A{2}.rSendControlSignals;
%% DATA            
            data = [  data  ; A{1}.pPos.Xd(1:3)'     A{1}.pPos.X(1:3)' ...
                              A{2}.pPos.Xd(1:3)'     A{2}.pPos.X(1:3)' zeros(6,1)'  ...
                              zeros(8,1)'  A{1}.pPar.Model_simp' t_atual];
            
            %         %   1 -- 3      4 -- 6     
            %         B{1}.pPos.Xd'  B{1}.pPos.X' 
            %
            %         %   7 -- 9     10 -- 12        13 -- 18
            %         B{2}.pPos.Xd'  B{2}.pPos.X' barL.pPos.X_load'
            %
            %         %  19 -- 26             27 -- 34          35
            %      B{1}.pPar.Model_simp  B{2}.pPar.Model_simp  toc(t) ];
            
%% EMERGÊNCIA
        drawnow
        if btnEmergencia == 1
            A{2}.pFlag.EmergencyStop = 1;
        end
    
        if btnEmergencia ~= 0 || A{2}.pFlag.EmergencyStop ~= 0 || A{1}.pFlag.EmergencyStop ~= 0
            disp('Pioneer stopping by  ');
            break;
        end   
            
        end
    end

    end
    
A{1}.rLand;
A{2}.rLand;
        
%% PLOT
B1_Xtil = data(:,1:3) - data(:,4:6);
B2_Xtil = data(:,7:9) - data(:,10:12);
% Load_Xtil = [data(:,1:3) - data(:,13:15) data(:,7:9) - data(:,16:18)];
% Load_Xtil(:,[3 6]) = Load_Xtil(:,[3 6]) - barL.pPar.l1;


% 
figure();
hold on;
grid on;
plot(data(:,35),B1_Xtil(:,1));
plot(data(:,35),B1_Xtil(:,2));
plot(data(:,35),B1_Xtil(:,3));
title('Erro de Posição Q1');
legend('Pos X','Pos Y','Pos Z');
xlabel('Tempo[s]');
ylabel('Erro [m]');

figure();
hold on;
grid on;
plot(data(:,35),B2_Xtil(:,1));
plot(data(:,35),B2_Xtil(:,2));
plot(data(:,35),B2_Xtil(:,3));
title('Erro de Posição Q2');
legend('Pos X','Pos Y','Pos Z');
xlabel('Tempo[s]');
ylabel('Erro [m]');
% 
% figure();
% hold on;
% grid on;
% plot(data(:,35),Load_Xtil(:,1));
% plot(data(:,35),Load_Xtil(:,2));
% plot(data(:,35),Load_Xtil(:,3));
% title('Erro de Posição L1');
% legend('Pos X','Pos Y','Pos Z');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');
% 
% figure();
% hold on;
% grid on;
% plot(data(:,35),Load_Xtil(:,4));
% plot(data(:,35),Load_Xtil(:,5));
% plot(data(:,35),Load_Xtil(:,6));
% title('Erro de Posição L2');
% legend('Pos X','Pos Y','Pos Z');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');


% % 
% % figure();
% % hold on;
% % grid on;
% % plot(data(:,1),data(:,2));
% % plot(data(:,13),data(:,14));
% % title('XY');
% % xlabel('X [m]');
% % ylabel('Y [m]');
% % 
% % figure();
% % subplot(411)
% % hold on;
% % grid on;
% % plot(data(:,35),data(:,19));
% % plot(data(:,35),data(:,7));
% % xlabel('Tempo[s]');
% % ylabel('Velocidade [m/s]');
% % legend('dX', 'dXd');
% % 
% % subplot(412)
% % hold on;
% % grid on;
% % plot(data(:,35),data(:,20));
% % plot(data(:,35),data(:,8));
% % xlabel('Tempo[s]');
% % ylabel('Velocidade [m/s]');
% % legend('dY', 'dYd');
% % 
% % subplot(413)
% % hold on;
% % grid on;
% % plot(data(:,35),data(:,21));
% % plot(data(:,35),data(:,9));
% % xlabel('Tempo[s]');
% % ylabel('Velocidade [m/s]');
% % legend('dZ', 'dZd');
% % 
% % subplot(414)
% % hold on;
% % grid on;
% % plot(data(:,35),data(:,24));
% % plot(data(:,35),data(:,12));
% % xlabel('Tempo[s]');
% % ylabel('Velocidade [rad/s]');
% % legend('phi', 'dphi');
% % 
% % 
figure();
sgtitle('Q1')
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,4));
plot(data(:,end),data(:,1));
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('X', 'Xd');

subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,5));
plot(data(:,end),data(:,2));
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('Y', 'Yd');

subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,6));
plot(data(:,end),data(:,3));
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('Z', 'Zd');


figure();
sgtitle('Q2')
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,10));
plot(data(:,end),data(:,7));
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('X', 'Xd');

subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,11));
plot(data(:,end),data(:,8));
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('Y', 'Yd');

subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,12));
plot(data(:,end),data(:,9));
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('Z', 'Zd');
% 
% figure();
% subplot(421)
% grid on;
% plot(data(:,35),data(:,19));
% xlabel('Tempo[s]');
% ylabel('$K_1$','interpreter','latex')
% legend(['$K_1 = $' num2str(data(end,19))],'interpreter','latex')
% 
% subplot(422)
% grid on;
% plot(data(:,35),data(:,20));
% xlabel('Tempo[s]');
% ylabel('$K_2$','interpreter','latex')
% legend(['$K_2 = $' num2str(data(end,20))],'interpreter','latex')
% 
% subplot(423)
% grid on;
% plot(data(:,35),data(:,21));
% xlabel('Tempo[s]');
% ylabel('$K_3$','interpreter','latex')
% legend(['$K_3 = $' num2str(data(end,21))],'interpreter','latex')
% 
% subplot(424)
% grid on;
% plot(data(:,35),data(:,22));
% xlabel('Tempo[s]');
% ylabel('$K_4$','interpreter','latex')
% legend(['$K_4 = $' num2str(data(end,22))],'interpreter','latex')
% 
% subplot(425)
% grid on;
% plot(data(:,35),data(:,23));
% xlabel('Tempo[s]');
% ylabel('$K_5$','interpreter','latex')
% legend(['$K_5 = $' num2str(data(end,23))],'interpreter','latex')
% 
% subplot(426)
% hold on;
% grid on;
% plot(data(:,35),data(:,24));
% xlabel('Tempo[s]');
% ylabel('$K_6$','interpreter','latex')
% legend(['$K_6 = $' num2str(data(end,24))],'interpreter','latex')
% 
% subplot(427)
% hold on;
% grid on;
% plot(data(:,35),data(:,25));
% xlabel('Tempo[s]');
% ylabel('$K_7$','interpreter','latex')
% legend(['$K_7 = $' num2str(data(end,25))],'interpreter','latex')
% 
% subplot(428)
% hold on;
% grid on;
% plot(data(:,35),data(:,26));
% xlabel('Tempo[s]');
% ylabel('$K_8$','interpreter','latex')
% legend(['$K_8 = $' num2str(data(end,26))],'interpreter','latex')

figure();
subplot(421)
grid on;
plot(data(:,35),data(:,27));
xlabel('Tempo[s]');
ylabel('$K_1$','interpreter','latex')
legend(['$K_1 = $' num2str(data(end,27))],'interpreter','latex')

subplot(422)
grid on;
plot(data(:,35),data(:,28));
xlabel('Tempo[s]');
ylabel('$K_2$','interpreter','latex')
legend(['$K_2 = $' num2str(data(end,28))],'interpreter','latex')

subplot(423)
grid on;
plot(data(:,35),data(:,29));
xlabel('Tempo[s]');
ylabel('$K_3$','interpreter','latex')
legend(['$K_3 = $' num2str(data(end,29))],'interpreter','latex')

subplot(424)
grid on;
plot(data(:,35),data(:,30));
xlabel('Tempo[s]');
ylabel('$K_4$','interpreter','latex')
legend(['$K_4 = $' num2str(data(end,30))],'interpreter','latex')

subplot(425)
grid on;
plot(data(:,35),data(:,31));
xlabel('Tempo[s]');
ylabel('$K_5$','interpreter','latex')
legend(['$K_5 = $' num2str(data(end,31))],'interpreter','latex')

subplot(426)
hold on;
grid on;
plot(data(:,35),data(:,32));
xlabel('Tempo[s]');
ylabel('$K_6$','interpreter','latex')
legend(['$K_6 = $' num2str(data(end,32))],'interpreter','latex')

subplot(427)
hold on;
grid on;
plot(data(:,35),data(:,33));
xlabel('Tempo[s]');
ylabel('$K_7$','interpreter','latex')
legend(['$K_7 = $' num2str(data(end,33))],'interpreter','latex')

subplot(428)
hold on;
grid on;
plot(data(:,35),data(:,34));
xlabel('Tempo[s]');
ylabel('$K_8$','interpreter','latex')
legend(['$K_8 = $' num2str(data(end,34))],'interpreter','latex')


%% Send control signals
A{2}.pSC.Ud = [0  ;  0];
for ii = 1:50
 r.SendROS('robot1/vel',A{2}.pSC.Ud);
end
