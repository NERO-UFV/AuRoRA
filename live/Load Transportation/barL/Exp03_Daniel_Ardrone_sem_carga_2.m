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
% A.pPar.ip = '192.168.1.30';
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

rX = .5;           % [m]
rY = .5;           % [m]
T = 10;             % [s]
Tf = T*2;            % [s]
w = 2*pi/T;         % [rad/s]

% timers
t_incB = tic;
t  = tic;
t_control = tic;
T_exp = 240; % tempo de experimento
T_run = 1/100; % período de amostragem do experimento
t_run = tic;
t_total = tic; % tempo que o experimento está rodando (running)
t_plot = tic; % 
T_plot = .3; % período de plotagem
t_task = tic;
T_task = 13;
i_task = 0;
t_exp = tic;

A.pPar.Ts = 1/100;


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

% Trajetória 2
%             if t_atual < T_exp
%                 A.pPos.Xd([1:3 6]) = [rX*sin(w*t_atual);
%                     rY*cos(0.5*w*t_atual);
%                     1.2 + 0.5*sin(w*t_atual);
%                     0];
% 
% 
%                 A.pPos.Xd([7:9 12]) = [w*rX*cos(w*t_atual);
%                     -0.5*w*rY*sin(0.5*w*t_atual);
%                     w*0.5*cos(w*t_atual);
%                     0];
%                 
%                 A.pPos.dXd([7:9 12]) = [-w^2*rX*sin(w*t_atual);
%                     -0.5^2*w^2*rY*cos(0.5*w*t_atual);
%                     -w^2*0.5^2*sin(w*t_atual);
%                     0];
%                 
%                 A.pPos.Xd(6) = atan2(A.pPos.Xd(8),A.pPos.Xd(7));
%                 A.pPos.Xd(12) = (A.pPos.Xd(6) - A.pPos.Xda(6))/toc(A.pPar.ti);
%                 A.pPos.Xda = A.pPos.Xd;
               
            end


            
%% Posição            
%         if toc(t_xd) > T_xd && Xd_i < 6
%             t_xd = tic;
%             Xd_i = Xd_i + 1;
% %             A.pPos.Xd(1:3) = Xd(1:3,Xd_i);
% %             A.pPos.Xd(6) = Xd(4,Xd_i);
%             A.pPos.Xd(1:3) = [0 0 1];
% %             A.pPos.Xd(6) = sin(w*t_atual);
% %             A.pPos.Xd(12) = cos(w*t_atual);
%         end
        



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
        %% If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
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
Xtil = data(:,1:12) - data(:,13:24);

figure();
hold on;
grid on;
plot(data(:,25),Xtil(:,1));
plot(data(:,25),Xtil(:,2));
plot(data(:,25),Xtil(:,3));
% plot(data(:,25),Xtil(:,6));
axis([0 70 -.2 .2])
title('Erro de Posição');
legend('Pos X','Pos Y','Pos Z', 'Ori Z');
xlabel('Tempo[s]');
ylabel('Erro [m]');

figure();
subplot(411)
hold on;
grid on;
plot(data(:,25),data(:,1));
plot(data(:,25),data(:,13));
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('X', 'Xd');

subplot(412)
hold on;
grid on;
plot(data(:,25),data(:,2));
plot(data(:,25),data(:,14));
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('Y', 'Yd');

subplot(413)
hold on;
grid on;
plot(data(:,25),data(:,3));
plot(data(:,25),data(:,15));
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('Z', 'Zd');

subplot(414)
hold on;
grid on;
plot(data(:,25),data(:,6));
plot(data(:,25),data(:,18));
xlabel('Tempo[s]');
ylabel('Velocidade [rad/s]');
legend('phi', 'phi_d');
% 
%     % Set maximum flight_without_shell 
%     fprintf(A.pCom.controlChannel, ...
%         sprintf( 'AT*%s=%i,%s\r', 'CONFIG', A.pCom.SequenceNumber, '"control:control_vz_max","1000"'));
%     A.pCom.SequenceNumber = A.pCom.SequenceNumber + 1;

% 
    %Set maximum flight_without_shell 
    fprintf(A.pCom.controlChannel, ...
        sprintf( 'AT*%s=%i,%s\r', 'CONFIG', A.pCom.SequenceNumber, '"control:euler_angle_max","0.26"'));
    A.pCom.SequenceNumber = A.pCom.SequenceNumber + 1;



