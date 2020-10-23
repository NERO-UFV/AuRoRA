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
A{1} = ArDrone(1);
A{2} = ArDrone(2);
A{1}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]'; %original
A{2}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]'; %original

%% Joystick Initialization
J = JoyControl;


%% Optitrack Initialization
OPT = OptiTrack;
OPT.Initialize;

% Initial Pose
idA{1} = getID(OPT,ArDrone,1);
idA{2} = getID(OPT,ArDrone,2);
    

% Envio de comando take off para os drones
A{1}.rConnect;
A{1}.rTakeOff;
A{2}.pPar.LocalPortControl = 5558;
A{2}.pPar.LocalPortState = 5552;
A{2}.rConnect;
A{2}.rTakeOff;

espera = 0;
% pause(5)

% Variáveis de trajetória
T_xd = 5;
t_xd = tic;
data = [];

rX = .5;           % [m]
rY = .5;           % [m]
T = 20;             % [s]
Tf = T*2;            % [s]
w = 2*pi/T;         % [rad/s]

% timers
t_incB = tic;
t  = tic;
t_control = tic;
T_exp = 480; % tempo de experimento
T_run = 1/100; % período de amostragem do experimento
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


A{1}.pPar.Ts = 1/100;
A{1}.pPar.ti = tic;
A{1}.pPar.counter = 0;

A{2}.pPar.Ts = 1/100;
A{2}.pPar.ti = tic;
A{2}.pPar.counter = 0;


    while toc(t) < T_exp 
        

        if toc(t_run) > T_run 
            
            t_run = tic;
            t_atual = toc(t);
            A{1}.pPar.counter = A{1}.pPar.counter + 1;
            A{2}.pPar.counter = A{1}.pPar.counter + 1;
                
        
%% Trajetória
% Trajetória 1

% Trajetória 2

%                 A{1}.pPos.Xd([1:3 6]) = [rX*sin(w*t_atual);
%                     rY*cos(0.5*w*t_atual);
%                     1.7 + 0.5*sin(w*t_atual);
%                     0];
% 
% 
%                 A{1}.pPos.Xd([7:9 12]) = [w*rX*cos(w*t_atual);
%                     -0.5*w*rY*sin(0.5*w*t_atual);
%                     w*0.5*cos(w*t_atual);
%                     0]; 
%                 
%                 A{1}.pPos.dXd([7:9 12]) = [-w^2*rX*sin(w*t_atual);
%                     -0.5^2*w^2*rY*cos(0.5*w*t_atual);
%                     -w^2*0.5*sin(w*t_atual);
%                     0]; 

            A{1}.pPos.Xd([1:3 6]) = [0;-1;1;0];
            A{1}.pPos.Xd([7:9 12]) = [0;0;0;0];
            A{1}.pPos.dXd([7:9 12]) = [0;0;0;0];
            
            A{2}.pPos.Xd([1:3 6]) = [0;1;2.5;0];
            A{2}.pPos.Xd([7:9 12]) = [0;0;0;0];
            A{2}.pPos.dXd([7:9 12]) = [0;0;0;0];
            
            if t_atual > 10
            A{1}.pPos.Xd([1:3 6]) = [0;0;1;0];
            A{2}.pPos.Xd([1:3 6]) = [0;0;2.5;0];
            end
            
            if t_atual > 30
            A{1}.pPos.Xd([1:3 6]) = [0;-1;1;0];
            A{2}.pPos.Xd([1:3 6]) = [0;1;2.5;0];
            end
            
            
            
% Posição            
%         if toc(t_xd) > T_xd && Xd_i < 6
%             t_xd = tic;
%             Xd_i = Xd_i + 1;
%             A{1}.pPos.Xd(1:3) = Xd(1:3,Xd_i);
%             A{1}.pPos.Xd(6) = Xd(4,Xd_i);
%         end
        



        %% Obter os dados dos sensores
        rb = OPT.RigidBody;
            if rb(idA{1}).isTracked
            A{1} = getOptData(rb(idA{1}),A{1});
%             A{1}.pPos.X
            end
            
            if rb(idA{2}).isTracked
            A{2} = getOptData(rb(idA{2}),A{2});
%             A{2}.pPos.X
            end
           
       
        
        %% Implementação do controle
        % Controlador

            
        if t_atual > 5
            A{1}.pSC.Control_flag = 1;
            A{2}.pSC.Control_flag = 1;
        end
        
%             A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});

               

        A{1} = J.mControl(A{1});                       % joystick command (priority)        
        A{2} = J.mControl(A{2});                       % joystick command (priority)
                
        A{1}.rSendControlSignals;
        A{2}.rSendControlSignals;

        %% If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            drawnow
            if btnEmergencia ~= 0 
                disp('Bebop Landing through Emergency Command ');

                % Send 3 times Commands 1 second delay to Drone Land
                for i=1:nLandMsg
                    disp("End Land Command");
                   A{1}.rLand;
                   A{2}.rLand;
                end
                break;
            end         
            
        % Coleta de dados
        data = [  data  ; A{1}.pPos.Xd' A{1}.pPos.X' t_atual A{1}.pPar.Model_simp'];
%                          1 -- 12    13 -- 24   25     26 -- 33

%                       
    end
    
%     subplot(3,1,3)
%     plot(data(:,1),data(:,49),data(:,13),data(:,49))



    end
    A{1}.rLand;
    A{2}.rLand;

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
ylabel('x [m]');
legend('Xd', 'X');

subplot(412)
hold on;
grid on;
plot(data(:,25),data(:,2));
plot(data(:,25),data(:,14));
xlabel('Tempo[s]');
ylabel('y [m]');
legend('Yd', 'Y');

subplot(413)
hold on;
grid on;
plot(data(:,25),data(:,3));
plot(data(:,25),data(:,15));
xlabel('Tempo[s]');
ylabel('z [m]');
legend('Zd', 'Z');

subplot(414)
hold on;
grid on;
plot(data(:,25),data(:,6));
plot(data(:,25),data(:,18));
xlabel('Tempo[s]');
ylabel('psi [rad]');
legend('phid', 'phi');

figure();
subplot(411)
hold on;
grid on;
plot(data(:,25),data(:,7));
plot(data(:,25),data(:,19));
axis([0 70 -.5 .5])
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('Xdot_des', 'Xdot');

subplot(412)
hold on;
grid on;
plot(data(:,25),data(:,8));
plot(data(:,25),data(:,20));
axis([0 70 -.5 .5])
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('Ydot_des', 'Ydot');

subplot(413)
hold on;
grid on;
plot(data(:,25),data(:,9));
plot(data(:,25),data(:,21));
axis([0 70 -.5 .5])
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('Zdot_des', 'Zdot');

subplot(414)
hold on;
grid on;
plot(data(:,25),data(:,12));
plot(data(:,25),data(:,24));
axis([0 70 -1.6 1.6])
xlabel('Tempo[s]');
ylabel('Velocidade [rad/s]');
legend('phidot_des', 'phidot');


figure();
hold on;
grid on;
plot(data(:,1),data(:,2));
plot(data(:,13),data(:,14));
title('XY');
xlabel('X [m]');
ylabel('Y [m]');

