%% POSITION CONTROL FOR PIONEER


clear
close all
clc
% Fecha todas possíveis conexões abertas
try
    fclose(instrfindall);
catch
end

%% Look for root directory
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_Position_Pioneer')
Arq = fopen(['PositionMobileSim_' NomeArq '.txt'],'w');
cd(PastaAtual)

%% Definição da janela do gráfico
fig = figure(1);
axis([-4 4 -4 4])
% axis equal

%% Classes initialization
% Robot
P = Pioneer3DX;

%% Conexão com robô/simulador
P.rConnect;             % robô ou mobilesim
pause(10);
disp('Início..............')

%% Posição inicial do robô
Xo = [0 0 0 0];
P.rSetPose([0 0 0 0]);    % define pose do robô
P.pPos.X(1:2) = [0 0];

% Desired pose [x y]
P.pPos.Xd([1 2 6]) = [0 0 pi/2];


%% Initial errors
P.pPos.Xtil = P.pPos.Xd - P.pPos.X;

%% Variables initialization
vmax = .75;   % pioneer maximum speed
kw   = 0.7;     % angular velocity gain
XX   = [];    % simulation data

%% Simulation

% Temporização
tap = 0.1;     % taxa de atualização do pioneer
tc = tic;
tp = tic;
t  = tic;      % simulation time



while toc(t) < 40
    toc(t)
    if toc(tc) > tap
        
        tc = tic;    % reset clock 
        
        if toc(t) > 10
            P.pPos.Xd(6) = -pi/2;
        end
        
        if toc(t) > 20
            P.pPos.Xd(6) = pi;
        end
        
        if toc(t) > 30
            P.pPos.Xd(6) = 0;
        end
        
        
        
        % ---------------------------------------------------------------
        % Data aquisition
        P.rGetSensorData;
        
        % --------------------------------------------------------------
        % Errors
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        disp(P.pPos.Xtil(6))
        
        % --------------------------------------------------------------
        % Control
        P.pSC.Ud(1) = 0;
        P.pSC.Ud(2) = 0.5*P.pPos.Xtil(6)^2/sin(P.pPos.Xtil(6));
        % -------------------------------------------------------------
        
        % Send control to robot
        P.rSendControlSignals;
        
        % Save data 
        XX = [XX; P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)];
        
%         % Txt file
%         fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' rho alfa theta toc(t)]);
%         fprintf(Arq,'\n\r');
        
    end
    
    %% Desenha os robôs
    
    if toc(tp) > tap
        tp = tic;
        %         try
        P.mCADdel
        delete(fig);
        
         P.mCADplot(1,'k')
%         plot(XX(:,1),XX(:,2),'kx');
%         hold on;
%         plot(XX(:,13),XX(:,14),'g');
        axis([-4 4 -4 4])
        grid on
        drawnow
    end
    
end

% Close txt file
fclose(Arq);

%%  Stop robot
% Zera velocidades do robô
P.pSC.Ud = [0 ; 0];
P.rSendControlSignals;

%% Results

% Position
figure;
plot(XX(:,1),XX(:,2),'kx','MarkerSize',10),hold on;
plot(XX(:,13),XX(:,14)), legend('Desejado','Percorrido')
grid on
% Errors
figure;
plot(XX(:,end),XX(:,29))
title('\rho [m]')
grid on

% figure;
% plot(XX(:,end),rad2deg(XX(:,30)))
% title('\alpha [^o]')
% grid on


% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
