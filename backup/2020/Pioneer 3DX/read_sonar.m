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
P.rConnect;             % robô ou mobilesim
% shg                   % show graphic
% pause(7)
disp('Start..............')

%% Initial Position
% Xo = input('Digite a posição inicial do robô ([x y z psi]): ');
Xo = [0 0 0 0];

P.rSetPose(Xo);         % define pose do robô
P.pSC.Ud =[0;0];        % zerar vels. desejadas

%% Variables initialization
% Xa = P.pPos.X(1:6);    % postura anterior
data = [];
Sdata = [];
Rastro.Xd = [];
Rastro.X = [];
SonarData = [];
LinMap = [];

%% Trajectory variables
a = 1.5;         % distância em x
b = 1;         % distância em y

w = 0.1;

nvoltas = 2;
tsim = 2*pi*nvoltas/w;
tsim = 60;

%% Simulation

% Temporização
tap = 0.1;     % taxa de atualização do pioneer
t = tic;
tc = tic;
tp = tic;

while toc(t) < tsim
    
    if toc(tc) > tap
        
        tc = tic;
        % Data aquisition
        P.rGetSensorData;
        SonarData = P.rGetSonarData;
        
        % Polar => cartesian transform:
        X = SonarData(2,:).*cos(SonarData(1,:));
        Y = SonarData(2,:).*sin(SonarData(1,:));
                
        % Goal setting:
        med = mean([X;Y]'); 
        P.pPos.Xd(1)= med(1);
        P.pPos.Xd(2)= med(2);
        
        % salva variáveis para plotar no gráfico
        Rastro.Xd = [Rastro.Xd; P.pPos.Xd(1:2)'];  % formação desejada
        Rastro.X  = [Rastro.X; P.pPos.X(1:2)'];    % formação real
         
        % Control
        P = fDynamicController(P,gains);
        Sdata = [Sdata;SonarData];
        data = [data; P.pSC.Ud' P.pSC.U'];
        
        % Send control to robot
        P.rSendControlSignals;
               
        
    end
    
    %% Desenha o robô
%     if toc(tp) > tap
%         tp = tic;
%        
%         P.mCADdel
%         delete(fig);
%         P.mCADplot(1,'k')
%         hold on
%         plot(med(1),med(2),'r*')
%         hold on
%         plot(X,Y,'b--');
%         hold on
%         plot(Rastro.Xd(:,1),Rastro.Xd(:,2),'k');
%         hold on;
%         plot(Rastro.X(:,1),Rastro.X(:,2),'black');
%         axis([-5 5 -5 5])
%         grid on
%         drawnow
%     end
    
end

%%  Stop robot
% Zera velocidades do robô
P.pSC.Ud = [0 ; 0];
P.rSendControlSignals;

save sonar_dados2.mat

P.rDisconnect

