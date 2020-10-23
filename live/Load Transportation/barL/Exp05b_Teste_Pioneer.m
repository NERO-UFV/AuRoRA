clear; close all; clc;
try
    fclose(instrfindall);
catch
end
rosshutdown;

%% Buscar pasta raiz
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))




%% Load Class
    % Load Classes
      % Create OptiTrack object and initialize
    OPT = OptiTrack;
    OPT.Initialize;
    
    % Initiate classes
    P = Pioneer3DX(1);
    idP = getID(OPT,P); % ID do Bebop

    
% Network
r = ROSNetwork;
r.InitROS('/master')
r.InitROS('/robot1','192.168.0.158')
r.PublisherROS(r.node,'robot1/vel');
P.pSC.Ud = [0 0]';
r.SendROS('robot1/vel',P.pSC.Ud);

%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'land', ...
                         'Callback', 'btnEmergencia=1', ...
                         'Position', [50 50 400 300]);

% Variáveis de trajetória
Xd = [0 0 0 0]';
dXd = [0 0 0 0]';

Xd_i = 1;
Xd(:,1) = [0 0 0 0]';
Xd(:,2) = [.5 .5 0 0]';
Xd(:,3) = [-.5 .5 0 0]';
Xd(:,4) = [-.5 -.5 0 0]';
Xd(:,5) = [.5 -.5 0 0]';
Xd(:,6) = [0 0 0 0]';
P.pPos.Xd(1:3) = Xd(1:3,Xd_i);
P.pPos.Xd(6) = Xd(4,Xd_i);
T_xd = 5;
t_xd = tic;
data = [];

rX = 1;           % [m]
rY = 1;           % [m]
T = 15;             % [s]
w = 2*pi/T;         % [rad/s]

% timers
t  = tic;
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

pgains = [1.5 1 1.5 1];
P.pPar.Ts = 1/30;
P.pPar.ti = tic;




    while toc(t) < T_exp 

        if toc(t_run) > T_run 
            
            t_run = tic;
            t_atual = toc(t);
                
        
%% Trajetória
                  Xd = [rX*sin(w*t_atual);
                    rY*cos(0.5*w*t_atual);
                    0;
                    0];
                
                
                dXd = [w*rX*cos(w*t_atual);
                    -0.5*w*rY*sin(0.5*w*t_atual);
                     0;
                    0];           

            
%% Posição            
%         if toc(t_xd) > T_xd && Xd_i < 6
%             t_xd = tic;
%             Xd_i = Xd_i + 1;
%             Xd(1:2) = Xd(1:2,Xd_i);
%             disp(P.pPos.Xd(1:2))
%             
%         end
        



        %% Obter os dados dos sensores
        rb = OPT.RigidBody;
            if rb(idP).isTracked
            P = getOptData(rb(idP),P);
%             P.pPos.X
            end
           
       
        
        %% Implementação do controle
        % Controlador
            P.pPos.Xd(1:2) = Xd(1:2);
                        
            P.pPos.Xd(7:8) = dXd(1:2);
            
            

            P = fControladorCinematico(P,pgains);

            P.pPar.ti = tic;
            r.SendROS('robot1/vel',P.pSC.Ud);
                
        % Coleta de dados
        data = [  data  ; P.pPos.Xd' P.pPos.X' t_atual];
%                          1 -- 12    13 -- 24   25     2
%                       
%% EMERGÊNCIA
        drawnow
        if btnEmergencia == 1
            P.pFlag.EmergencyStop = 1;
        end
    
        if btnEmergencia ~= 0 || P.pFlag.EmergencyStop ~= 0 
            disp('Pioneer stopping by  ');

            % Send 3 times Commands 1 second delay to Drone Land
            for i=1:nLandMsg
                P.pSC.Ud = [0 0]';
                r.SendROS('robot1/vel',P.pSC.Ud);
            end
            break;
        end         
   

        end
    end

%% EMERGÊNCIA FORA DO LAÇO    
    for i=1:nLandMsg
        P.pSC.Ud = [0 0]';
        r.SendROS('robot1/vel',P.pSC.Ud);
    end
    

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
hold on;
grid on;
plot(data(:,1),data(:,2));
plot(data(:,13),data(:,14));
title('XY');
xlabel('X [m]');
ylabel('Y [m]');

%% Send control signals
P.pSC.Ud = [0 0]';
for ii = 1:50
 r.SendROS('robot1/vel',P.pSC.Ud);
end
