%% CONTROLE DE TRAJETÓRIA PARA PIONEER UTILIZANDO OPTITRACK
%  Este algoritmo deve rodar no pc do optitrack, que fará todo
%  processamento e enviará para rede os sinais de controle para o pioneer.
% *Este pc precisará receber os dados dos sensores de velocidade do pioneer


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

%% Graphic window definition
fig = figure(1);
axis([-3 3 -3 3])
axis equal
grid on
hold on

%% Classes initialization
% Robot
P = Pioneer3DX;
P.pPar.a = 0.08;   % control point
% gains format: [Kcinematico1(1:2) Kcinematico2(1:2) Kdinamico1(1:2) Kdinamico2(1:2)]
gains = [0.3 0.3 0.4 0.4 0.7 0.3 0.1 0.01];

% Network
Rede = NetDataShare;

% OptiTrack
OPT = OptiTrack;
OPT.Initialize;


% Joystick control
% J = JoyControl;

%% Define robot initial position
idP = getID(OPT,P);         % pioneer ID on optitrack
rb = OPT.RigidBody;
% n = length(rb);

P = getOptData(rb(idP),P);

P.rSetPose(P.pPos.X([1 2 3 6]));         % define pose do robô

%% Network communication check
tm = tic;
while true
    
    if isempty(Rede.pMSG.getFrom)   % no message received at all
        Rede.mSendMsg(P);
        if toc(tm) > 0.1
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        end
        
    elseif length(Rede.pMSG.getFrom) > 1   % if client data received
        Rede.mSendMsg(P);
        if isempty(Rede.pMSG.getFrom{2})
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        else
            break
        end
    end
end
clc
disp('Data received. Segue o jogo...');

%% Inicialização de variáveis
% Xa = P.pPos.X(1:6);    % postura anterior
data = [];
Rastro.Xd = [];
Rastro.X = [];
Rastro.O = [];
Rastro.U  = [];
Rastro.Ud = [];
Rastro.Xc = [];
k = 0;
X=[];

dt = 1/30;
%% Trajectory variables
a = 1.5;         % distância em x
b = 1;         % distância em y

w = 0.1;

nvoltas = 2;
tsim = 2*pi*nvoltas/w;
% tsim = 60;

%% Simulation

% Time variables
tap = 0.1;     % taxa de atualização do pioneer
% timeout = 200;   % tempo máximo de duração da simulação
t = tic;
tc = tic;
tp = tic;
td = tic;

while toc(t) < tsim
    
    if toc(tc) > 1/30
        
        tc = tic;
        % ---------------------------------------------------------------
        % Data aquisition
        % Get network data
        Rede.mReceiveMsg;
        % If there is data from optitrack, overwrite position values
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{2}(29:30); % current velocities (robot sensors)
            X  = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
            %             disp('Velocity received!');
        else
            k = k+1;
            disp('No data received from robot......')
        end
        
        % Position data from optitrack
        rb = OPT.RigidBody;             % read optitrack
        P = getOptData(rb(idP),P);
        
        
        % Trajectory -------------------------------------------------------
        % infinito (8')
        ta = toc(t);
        P.pPos.Xd(1)  = a*sin(w*ta);       % posição x
        P.pPos.Xd(2)  = b*sin(2*w*ta);     % posição y
        P.pPos.Xd(7)  = a*w*cos(w*ta);     % velocidade em x
        P.pPos.Xd(8)  = 2*b*w*cos(2*w*ta); % velocidade em y
        
        % --------------------------------------------------------------
        % Control        
%         P = fControladorCinematico(P);
%         % Dynamic compensation
%         P = fCompensadorDinamico(P);

        P = fDynamicController(P,gains);
        % #####################################################
        % Descomentar quando não usar compensação dinâmica
        %         P.pSC.Ud = P.pSC.Ur;
        % #####################################################
        
        
        Rede.mSendMsg(P);  % send data to network
        
        % -------------------------------------------------------------
        % save data to plot
        Rastro.Xc = [Rastro.Xc; P.pPos.Xc(1:2)'];
        Rastro.Xd = [Rastro.Xd; P.pPos.Xd(1:2)'];  % desired trajectory
        Rastro.O  = [Rastro.O; P.pPos.X(1:2)'];    % optitrack position
        Rastro.X  = [Rastro.X; X'];                % robot sensors position
        Rastro.Ud = [Rastro.Ud; P.pSC.Ud'];        % control signal
        Rastro.U = [Rastro.U; P.pSC.U'];           % current velocities
        
        % Desenha os robôs
        if toc(tp) > tap
            tp = tic;
            P.mCADdel
            delete(fig);
            P.mCADplot(1,'k')
            plot(Rastro.Xd(:,1),Rastro.Xd(:,2),'k'),hold on;
            plot(Rastro.O(:,1),Rastro.O(:,2),'g');
            axis([-3 3 -3 3])
            drawnow
        end
        
    end
    
end

%%  Stop robot
% define control signal null
P.pSC.Ud = [0 ; 0];
% Send to network (a few times to be sure)
for ii = 1:5
    Rede.mSendMsg(P);
end
%% Results

% Control Signals
figure;
subplot(211)
plot(Rastro.Ud(:,1)),hold on;
plot(Rastro.U(:,1)), legend('u_d','u_r')
grid on
subplot(212)
plot(Rastro.Ud(:,2)),hold on
plot(Rastro.U(:,2)), legend('\omega_d','\omega_r')
grid on

% Trajectory
figure;
plot(Rastro.Xd(:,1),Rastro.Xd(:,2),'k','LineWidth',1.5),hold on
plot(Rastro.X(:,1),Rastro.X(:,2),'b--')
plot(Rastro.O(:,1),Rastro.O(:,2),'r-')
plot(Rastro.Xc(:,1),Rastro.Xc(:,2),'g-.')
legend('Desejado','Sensor','Optitrack','Xc')
grid on

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
