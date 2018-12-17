%% CONTROLE DE TRAJETÓRIA PARA PIONEER UTILIZANDO OPTITRACK

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

% %% Definição da janela do gráfico
% fig = figure(1);
% axis([-5 5 -5 5])
% % axis equal

%% Classes initialization
% Robot
P = Pioneer3DX;

% Network
Rede = NetDataShare;

% OptiTrack
OPT = OptiTrack;
OPT.Initialize;

% Joystick control
% J = JoyControl;

%% Posição inicial do robô
% Xo = input('Digite a posição inicial do robô ([x y psi]): ');
% Xo = [0 0 0 0];
% P.pPos.X([1 2 6]) = [Xo(1) ; Xo(2); Xo(3)];
rb = OPT.RigidBody;
n = length(rb);
if ~isempty(n)
    for ii = 1:n
        switch rb(ii).Name(1)
            case 'P'
                P = getOptData(rb(ii),P);
        end
    end
else
    disp('No rigid body tracked');
end

%% Conexão com robô/simulador
% P.rConnect;             % robô ou mobilesim
% P.rSetPose([-1 0 0 0]);         % define pose do robô
% shg                   % show graphic
% pause(3)
disp('Início..............')

%% Network communication

tm = tic;
while length(Rede.pMSG.getFrom)<=1
    Rede.mSendMsg(P);
    %     if toc(tm) > 0.1
    tm = tic;
    Rede.mReceiveMsg;
    disp('Waiting for message......')
    %     end
end
clc
disp('Data received......');

%% Inicialização de variáveis
% Xa = P.pPos.X(1:6);    % postura anterior
data = [];
Rastro.Xd = [];
Rastro.X = [];
k = 0;   % package received counter

% Ganhos do controlador
% K1 = diag([0.5 0.01]);      % limitador do sinal
% K2 = diag([1 1]);

K1 = 0.01;      % limitador do sinal
K2 = 0.1;
%% Position variables
% Desired Positions
Xd = [  0    0    ;
        0.4   0    ;
        0    0    ;
        0.4  0    ];

cont = 0;    % counter

%% Simulation

% Temporização
tsim = 100;
tap = 0.1; 
t = tic;
tc = tic;
tp = tic;

while toc(t) < tsim
    
    if toc(tc) > 0.1
        
        tc = tic;
        
         % ------------------------------------------------------------
         % Desired Position
            if toc(t) > cont*10
                
                cont = cont + 1;
                
            end
            
            if cont<=length(Xd)
                P.pPos.Xd(1) = Xd(cont,1);   % x
                P.pPos.Xd(2) = Xd(cont,2);   % y
            end
        % ---------------------------------------------------------------
        % Data aquisition
        % ####################################
        % For Test purpose without network only
        %   P.rGetSensorData;
        % #####################################
        
        % Position data from optitrack
        rb = OPT.RigidBody;
        n = length(rb);
        if ~isempty(rb)
            for ii = 1:n
                switch rb(ii).Name(1)
                    case 'P'
                        P = getOptData(rb(ii),P);
                end
            end
        else
            disp('No rigid body tracked.');
        end
        
        % salva variáveis para plotar no gráfico
        Rastro.Xd = [Rastro.Xd; P.pPos.Xd(1:2)'];  % formação desejada
        Rastro.X  = [Rastro.X; P.pPos.X(1:2)'];    % formação real
       
        % Real velocities (from robot sensors via network, used for dynamic compensation)
        Rede.mReceiveMsg;
        
        if length(Rede.pMSG.getFrom)>1
            %          % Assign variables
            %             P.pPos.Xd = Rede.pMSG.getFrom{2}(2+(1:12));
            %             P.pPos.X = Rede.pMSG.getFrom{2}(14+(1:12));
            P.pSC.U  =  Rede.pMSG.getFrom{2}(29:30);  % real velocities
            %             disp('Velocity received')
        end
        
        
        % --------------------------------------------------------------
        % Control
        
        % Error calculation
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        % Quadrant correction
        if abs(P.pPos.Xtil(6)) > pi
            if P.pPos.Xtil(6) > 0
                P.pPos.Xtil(6) = -2*pi + P.pPos.Xtil(6);
            else
                P.pPos.Xtil(6) =  2*pi + P.pPos.Xtil(6);
            end
        end
        
        % Controller
        P.pPos.dXr = P.pPos.Xd + K1*tanh(K2*P.pPos.Xtil);
        
        % Get control signal
        P.sInvKinematicModel(P.pPos.dXr(1:2));
        
        % Dynamic compensation
        P = fCompensadorDinamico(P);
        
        % #####################################################
        % Descomentar quando estiver sem compensação dinâmica
        %          P.pSC.Ud = P.pSC.Ur;
        % #####################################################
        % -------------------------------------------------------------
        
        % Send control to robot
        %         P.rSendControlSignals;
        
        P.pSC.Ud'
%         data = [data P.pSC.Ud];
         data = [data; P.pSC.Ud' P.pSC.U'];
        %% Send data to network
        Rede.mSendMsg(P);
        
    end
    
    % Desenha os robôs
    %
    %     if toc(tp) > tap
    %         tp = tic;
    %         %         try
    %         P.mCADdel
    %         delete(fig);
    %
    %         %         catch
    %         %         end
    %         %             P.mCADplot2D('k')
    %         P.mCADplot(1,'k')
    %         hold on
    %         fig1 = plot(Rastro.Xd(:,1),Rastro.Xd(:,2),'k');
    %         fig2 = plot(Rastro.X(:,1),Rastro.X(:,2),'g');
    %         axis([-5 5 -5 5])
    %         grid on
    %         drawnow
    %     end
    
end

%%  Stop robot
% Zera velocidades do robô
P.pSC.Ud = [0 ; 0];
% P.rSendControlSignals;
Rede.mSendMsg(P);

%% Results

% Control Signals
figure;
subplot(211)
plot(data(:,1)),hold on;
plot(data(:,3)), legend('Ud Linear ','Real')
grid on
subplot(212)
plot(data(:,2)),hold on
plot(data(:,4)), legend('Ud Angular','Real')
grid on
% Trajectory
figure;
plot(Rastro.Xd(:,1),Rastro.Xd(:,2),'x','MarkerSize',12),hold on
plot(Rastro.X(:,1),Rastro.X(:,2)),legend('Desejado','Percorrido')
grid on

% subplot(211)
% plot(data(1,:)), legend('Linear desejada')
% subplot(212)
% plot(data(2,:)), legend('Angular desejada')
% figure;
% plot(Rastro.Xd(:,1),Rastro.Xd(:,2)),hold on
% plot(Rastro.X(:,1),Rastro.X(:,2)),legend('Desejado','Percorrido')


% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
