%% CONTROLE DE TRAJETÓRIA PARA PIONEER

clear
close all
clc
% Fecha todas possíveis conexões abertas
try
    fclose(instrfindall);
catch
end

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


%% Open file to save data
% NomeArq = datestr(now,30);
% cd('DataFiles')
% cd('Log_Pioneer 3DX')
% Arq = fopen(['Traj_' NomeArq '.txt'],'w');
% cd(PastaAtual)


%% Graphic Window definition
fig = figure(1);
axis([-4 4 -4 4])
axis equal

%% Classes initialization
% Robot
P = Pioneer3DX;
P.pPar.a = 0.08;
% gains = [Kcinematico1(1:2) Kcinematico2(1:2) Kdinamico1(1:2) Kdinamico2(1:2)]
gains = [0.35 0.35 0.8 0.5 0.75 0.4 0.12 0.035];
% Joystick control
% J = JoyControl;

%% Connection with robot/simulator
% P.rConnect;             % robô ou mobilesim
% shg                   % show graphic
% pause(7)
disp('Start..............')

%% Initial Position
% Xo = input('Digite a posição inicial do robô ([x y z psi]): ');
Xo = [0 0 0 0];

P.rSetPose(Xo);         % define pose do robô

%% Variables initialization
% Xa = P.pPos.X(1:6);    % postura anterior
data = [];
Rastro.Xd = [];
Rastro.X = [];

%% Trajectory variables
a = 1.5;         % distância em x
b = 1;         % distância em y

w = 0.1;

nvoltas = 2;
tsim = 2*pi*nvoltas/w;

%% Simulation

% Temporização
tap = 0.1;     % taxa de atualização do pioneer
t = tic;
tc = tic;
tp = tic;

while toc(t) < tsim
    
    if toc(tc) > tap
        
        tc = tic;
        
        % Trajectory 
        % Lemniscata (8')
        ta = toc(t);
        P.pPos.Xd(1)  = a*sin(w*ta);       % posição x
        P.pPos.Xd(2)  = b*sin(2*w*ta);     % posição y
        P.pPos.Xd(7)  = a*w*cos(w*ta);     % velocidade em x
        P.pPos.Xd(8)  = 2*b*w*cos(2*w*ta); % velocidade em y
              
        % Data aquisition
        P.rGetSensorData;
        
        % salva variáveis para plotar no gráfico
        Rastro.Xd = [Rastro.Xd; P.pPos.Xd(1:2)'];  % formação desejada
        Rastro.X  = [Rastro.X; P.pPos.X(1:2)'];    % formação real
        
        % Control
        
        P = fDynamicController(P,gains);

        % Save data (.txt file)
%         fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
%             toc(t)]);
%         fprintf(Arq,'\n\r');
%         
        data = [data; P.pSC.Ud' P.pSC.U'];
        
        % Send control to robot
        P.rSendControlSignals;
               
        
    end
    
    %% Desenha o robô
    
    if toc(tp) > tap
        tp = tic;
       
        P.mCADdel
        delete(fig);
        P.mCADplot(1,'k')
        plot(Rastro.Xd(:,1),Rastro.Xd(:,2),'k');
        hold on;
        plot(Rastro.X(:,1),Rastro.X(:,2),'g');
        axis([-5 5 -5 5])
        grid on
        drawnow
    end
    
end

%%  Stop robot
% Zera velocidades do robô
P.pSC.Ud = [0 ; 0];
P.rSendControlSignals;

%% Close data file
% fclose(Arq);

%% Results

% Control Signals
figure;
subplot(211)
plot(data(:,1)),hold on;
plot(data(:,3)), legend('u_d','u_r')
subplot(212)
plot(data(:,2)),hold on
plot(data(:,4)), legend('\omega_d','\omega_r')

% Trajectory
figure;
plot(Rastro.Xd(:,1),Rastro.Xd(:,2)),hold on
plot(Rastro.X(:,1),Rastro.X(:,2)),legend('Desejado','Percorrido')



% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
