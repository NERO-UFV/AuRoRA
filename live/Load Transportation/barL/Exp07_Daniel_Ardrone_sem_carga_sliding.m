close all; clear; clc

try
    fclose(instrfindall);
catch
end

%% Buscar pasta raiz
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))




%% Robot Initialization
A = ArDrone(1);
A.pPar.Ts = 1/30;
A.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]'; %original
 
mCADcolor(A,[1 0 0])


%% Joystick Initialization
J = JoyControl;


%% Optitrack Initialization
OPT = OptiTrack;
OPT.Initialize;

% Initial Pose
idA = getID(OPT,ArDrone);

% Envio de comando take off para os drones
A.rConnect;
A.rTakeOff;
espera = 0;
% pause(5)

% Variáveis de trajetória
Xd = [0 0 0 0]';
dXd = [0 0 0 0]';
Xd_i = 1;
Xd(:,1) = [0 0 1 0]';
Xd(:,2) = [.5 .5 1.5 0]';
Xd(:,3) = [-.5 .5 2 0]';
Xd(:,4) = [-.5 -.5 0.5 0]';
Xd(:,5) = [.5 -.5 1 0]';
Xd(:,6) = [0 0 2 0]';
A.pPos.Xd(1:3) = Xd(1:3,Xd_i);
A.pPos.Xd(6) = Xd(4,Xd_i);
T_xd = 5;
t_xd = tic;
data = [];

rX = 1.01;           % [m]
rY = 1.01;           % [m]
T = 20;             % [s]
Tf = T*2;            % [s]
w = 2*pi/T;         % [rad/s]

% timers
t_incB = tic;
t  = tic;
t_control = tic;
T_exp = 120; % tempo de experimento
T_run = 1/30; % período de amostragem do experimento
t_run = tic;
t_total = tic; % tempo que o experimento está rodando (running)
t_plot = tic; % 
T_plot = .3; % período de plotagem
t_task = tic;
T_task = 13;
i_task = 0;
t_exp = tic;

%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);




    while toc(t) < T_exp 
        

        if toc(t_run) > T_run 
            
            t_run = tic;
            t_atual = toc(t);
                
        
%% Trajetória
% Trajetória 1
% Ciclo 1
%             if t_atual < 5
%                 Xd = [0;
%                     0;
%                     1.075;
%                     0];
%                 
%                 
%                 dXd = [0;
%                     0;
%                      0;
%                     0];
%             end
%  
% % Ciclo 2
%             if t_atual > 5 && t_atual < (5 + T_exp)
%                 Xd = [rX*sin(w*(t_atual + T/4 - 5));
%                     rY*cos(w*(t_atual + T/4 - 5));
%                     1.075 - 0.275*cos(w*(t_atual + T/4 - 5));
%                     0];
%                 
%                 
%                 dXd = [w*rX*cos(w*(t_atual + T/4 - 5));
%                     -w*rY*sin(w*(t_atual + T/4 - 5));
%                      0.275*w*sin(w*(t_atual + T/4 - 5));
%                     0];
%         end

% % Trajetória 2
            if t_atual < T_exp
                Xd = [rX*sin(w*t_atual);
                    rY*cos(0.5*w*t_atual);
                    1.7 + 0.5*sin(w*t_atual);
                    0];


                dXd = [w*rX*cos(w*t_atual);
                    -0.5*w*rY*sin(0.5*w*t_atual);
                    w*0.5*cos(w*t_atual);
                    0];

            end
            A.pPos.Xd(1:3) = Xd(1:3);
            A.pPos.Xd(6) = Xd(4);
            
            A.pPos.Xd(7:9) = dXd(1:3);
            A.pPos.Xd(12) = dXd(4);            

            
%% Posição            
%         if toc(t_xd) > T_xd && Xd_i < 6
%             t_xd = tic;
%             Xd_i = Xd_i + 1;
%             A.pPos.Xd(1:3) = Xd(1:3,Xd_i);
%             A.pPos.Xd(6) = Xd(4,Xd_i);
%         end
%         



        %% Obter os dados dos sensores
        rb = OPT.RigidBody;
            if rb(idA).isTracked
            A = getOptData(rb(idA),A);
%             A.pPos.X
            end
           
       
        
        %% Implementação do controle
        % Controlador

            
        if size(data,1) > 190 && A.pSC.Control_flag == 0 
            A.pSC.Control_flag = 1;
        end
        
            A = cInverseDynamicController_Compensador_ArDrone(A);
%             A = cInverseDynamicController_Adaptativo_ArDrone(A);
               

                
        A = J.mControl(A);                       % joystick command (priority)
                
        A.rSendControlSignals;

                    
            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            drawnow
            if btnEmergencia ~= 0 
                disp('Bebop Landing through Emergency Command ');

                % Send 3 times Commands 1 second delay to Drone Land
                for i=1:nLandMsg
                    disp("End Land Command");

                    A.rLand;
                end
                break;
            end
            
        % Coleta de dados
        data = [  data  ; A.pPos.Xd' A.pPos.X' t_atual A.pPar.Model_simp'];
%                          1 -- 12    13 -- 24   25     26 -- 33

%                       
    
    end
    
%     subplot(3,1,3)
%     plot(data(:,1),data(:,49),data(:,13),data(:,49))



    end
    A.rLand;

%% Plot results
nLandMsg = 3;
for i = 1:nLandMsg
    A.rLand;
end
Xtil = data(:,1:12) - data(:,13:24);

figure();
hold on;
grid on;
plot(data(:,25),Xtil(:,1));
plot(data(:,25),Xtil(:,2));
plot(data(:,25),Xtil(:,3));
% plot(data(:,25),Xtil(:,6));
axis([0 70 -.5 .5])
title('Erro de Posição');
legend('Pos X','Pos Y','Pos Z', 'Ori Z');
xlabel('Tempo[s]');
ylabel('Erro [m]');

figure();
hold on;
grid on;
plot(data(:,1),data(:,2));
plot(data(:,13),data(:,14));
title('XY');
xlabel('X [m]');
ylabel('Y [m]');

figure();
subplot(421)
grid on;
plot(data(:,25),data(:,26));
xlabel('Tempo[s]');
ylabel('$K_1$','interpreter','latex')
legend(['$K_1 = $' num2str(data(end,26))],'interpreter','latex')

subplot(422)
grid on;
plot(data(:,25),data(:,27));
xlabel('Tempo[s]');
ylabel('$K_2$','interpreter','latex')
legend(['$K_2 = $' num2str(data(end,27))],'interpreter','latex')

subplot(423)
grid on;
plot(data(:,25),data(:,28));
xlabel('Tempo[s]');
ylabel('$K_3$','interpreter','latex')
legend(['$K_3 = $' num2str(data(end,28))],'interpreter','latex')

subplot(424)
grid on;
plot(data(:,25),data(:,29));
xlabel('Tempo[s]');
ylabel('$K_4$','interpreter','latex')
legend(['$K_4 = $' num2str(data(end,29))],'interpreter','latex')

subplot(425)
grid on;
plot(data(:,25),data(:,30));
xlabel('Tempo[s]');
ylabel('$K_5$','interpreter','latex')
legend(['$K_5 = $' num2str(data(end,30))],'interpreter','latex')

subplot(426)
hold on;
grid on;
plot(data(:,25),data(:,31));
xlabel('Tempo[s]');
ylabel('$K_6$','interpreter','latex')
legend(['$K_6 = $' num2str(data(end,31))],'interpreter','latex')

subplot(427)
hold on;
grid on;
plot(data(:,25),data(:,32));
xlabel('Tempo[s]');
ylabel('$K_7$','interpreter','latex')
legend(['$K_7 = $' num2str(data(end,32))],'interpreter','latex')

subplot(428)
hold on;
grid on;
plot(data(:,25),data(:,33));
xlabel('Tempo[s]');
ylabel('$K_8$','interpreter','latex')
legend(['$K_8 = $' num2str(data(end,33))],'interpreter','latex')
