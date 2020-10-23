%% Controle de Formação Baseado em Espaço Nulo
% ICUAS 2019
% Mauro Sérgio Mafra e Sara Jorge e Silva

%
% Referência da formação: Pionner
%
% Tarefa prioritária sendo controle de forma    (subíndice f) 
% Tarefa secundária sendo o controle de posição (subíndice p)
%
% Tarefa 1: qp = [rhof alphaf betaf]'   Jf(x): 3 últimas linhas
% Tarefa 2: qf = [xf yf zf]'            Jp(x): 3 primeiras linhas
% Projeção no espaço nulo da primeira tarefa: (I-(invJf*Jf))
% Sendo J(x) a Jacobiana da transformação direta e invJ sua pseudo inversa
% 
% Lei de Controle 1: qRefPontoi = qPontoi + Ki*qTili
% Lei de Controle 2: qRefPontoi = qPontoi + Li*tanh(pinv(Li)*Ki*qTili)
% Para formação desejada constante -> qPontoi = 0
%

% Resetar 
clear all;   
close all;
warning off; 
clc;

%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

try
    fclose(instrfindall);
end

%% Load Class
try     
    % Load Classes
    P = Pioneer3DX(1);  % Pioneer Instance
    
    RI = RosInterface;
    RI.rConnect('192.168.0.147');
    %B = Bebop(1,'B');
    B = Bebop(1,'B'); 
    
    NSBF = NullSpace3D;  % Null Space Object Instance
    
    % Joystick
    J = JoyControl;
       
    disp('################### Load Class Success #######################');        
    
catch ME
    disp(' ');
    disp(' ################### Load Class Issues #######################');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    rosshutdown;
    return;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%% Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);

btnEmergencia = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Drone %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Condicoes iniciais
x(1) = 0;
y(1) = 0;
z(1) = 1;
phi(1) = 0;


%%%%%%%%%%%%%%%%%%%%%% Parametros do Teste %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Configuracao do teste
tmax = 50; % Tempo Simulacao em segundos

% Tempo de Amostragem
To = 1/5;

%t  = tic; % Tempo de simulacao
tc = tic; % Tempo de controle
tp = tic; % Tempo de plotagem e exibiï¿½ï¿½o
t  = tic;
tsw = tic;
t_xdp = tic;

B.pPar.ti = tc;
P.pPar.ti = tc;
nLandMsg = 3;    % Numero de mensagens enviadas para pouso do drone
dLandMsg = 1; % Atraso de tempo entre messagens de pouso

%% Decolagem do Drone

% Assegura a decolagem vertical
% Os valores de comando na decolagem sejam iguais a zero
B.pSC.Ud(1) = 0; % Esquerda/Direita [-1,1] (-) Move Drone para Esquerda
B.pSC.Ud(2) = 0; % Frente/Tras [-1,1] (-) Avanï¿½a - Move frente para baixo
B.pSC.Ud(3) = 0; % Velocidade Vertical [-1,1] (+) Eleva o drone
B.pSC.Ud(4) = 0; %

% Get Bebop Odometry
B.rGetSensorDataOpt;

% Inicializa o tempo de simulação
classArray = [];

data = [];
dataD = [];
comandos = [];
comandosP = [];
poses = [];
velocidades = [];
tempo = [];
Bebop = [];
dists=[];
xds=[];
xdps=[];

pPosAnt = B.pPos.X(6);
xd(4) = B.pPos.X(6);

tolerance = 0.1;
dist = 1000;
% mult = 2;
% mult2 = 0.1;
% xd = [mult*cos(mult2*toc(t)) mult*sin(mult2*toc(t)) 1 B.pPos.X(6)]';


xdp = [0 0 0 0];
xp = [0 0 0 0];

rX = 1.25;           % [m]
rY = 1.25;           % [m]
T = 45/2;             % [s]
Tf = 45;            % [s]
w = 2*pi/T;         % [rad/s]

disp(" ");
disp("Estabilization completed .... Turn ON Controller...")
disp(" ");

% Inicilize Drone Object Parameters
sampleTime = 1/30;
B.pPar.Ts = 1/30;

% Ajusted Gains Melhor Resultado Controlador Convencional
% NSBF.pPar.K1 =   2*diag([1 1 1 1 0.5 0.5]);                  % kinematic control gain - controls amplitude
% NSBF.pPar.K2 =   1*diag([1 1 1 1 1 1]);                  % kinematic control gain - control saturation

% Ajusted Gains
NSBF.pPar.K1 =   diag([1 1 1 2 4 7]);                  % kinematic control gain - controls amplitude
NSBF.pPar.K2 =   1*diag([1 1 1 1 1 1]);                  % kinematic control gain - control saturation

cGains = [ 0.1  0.1  0.75  0.75  0.75  0.75  0.10  0.03 ];
pgains = [0.2 0.2 1 1];


%% Desired Position 
Qd = [ 0   0.5   0   0.8  pi   pi/2 ];


% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Network
Rede = NetDataShare;


%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_NSB')
Arq = fopen(['NSB_Pos_Bebop_' NomeArq '.txt'],'w');
cd(PastaAtual)

%% Configure simulation window
fig = figure(2);

% plot robots initial positions
%plot3(P.pPos.X(1),P.pPos.X(2),P.pPos.X(3),'r^','LineWidth',0.8); hold on;
%plot3(B.pPos.X(1),B.pPos.X(2),B.pPos.X(3),'b^','LineWidth',0.8);

% Formation Weighted Error 
NSBF.mFormationWeightedError;

for kk = 1:size(Qd,1)
    % 
    NSBF.pPos.Qd = Qd(kk,:)';
    
    % Get Inverse Matrix 
    NSBF.mInvTrans;
    
    plot3(Qd(:,1),Qd(:,2),Qd(:,3),'r.','MarkerSize',20,'LineWidth',2),hold on;
    plot3(NSBF.pPos.Xd(4),NSBF.pPos.Xd(5),NSBF.pPos.Xd(6),'b.','MarkerSize',20,'LineWidth',2);
    
    % plot  formation line
    xl = [NSBF.pPos.Xd(1)   NSBF.pPos.Xd(4)];
    yl = [NSBF.pPos.Xd(2)   NSBF.pPos.Xd(5)];
    zl = [NSBF.pPos.Xd(3)   NSBF.pPos.Xd(6)];
    
    pl = line(xl,yl,zl);
    pl.Color = 'k';
    pl.LineStyle = '--';
    pl.LineWidth = 1;
    
    axis([-4 4 -4 4 0 3]);
    view(75,15)
    % Draw robot
    % P.mCADplot(1,'k');             % Pioneer
    % A.mCADplot                     % ArDrone
    grid on
    drawnow
end

%% Formation initial error

% Formation initial pose
NSBF.mDirTrans;

% Formation Error
NSBF.mFormationError;

% Formation Weighted Error 
NSBF.mFormationWeightedError;

%% First desired position
NSBF.pPos.Qd = Qd(1,:)';

% Robots desired pose
NSBF.mInvTrans;

% Formation Error
NSBF.pPos.Qtil = NSBF.pPos.Qd - NSBF.pPos.Q;


%% Simulation
% Maximum error permitted
% erroMax = [.1 .1 0 .1 deg2rad(5) deg2rad(5)];

% Time variables initialization
timeout = 150;   % maximum simulation duration
index   = 1;     % Index Counter
time    = 60;    % time to change desired positions [s]

% Get optitrack data
rb = OPT.RigidBody;             % read optitrack
% Ardrone
% B = getOptData(rb(idB),B);
idP = getID(OPT,P);         % Pioneer ID on optitrack
P = getOptData(rb(idP),P);

idB = getID(OPT,B);         % Pioneer ID on optitrack
B = getOptData(rb(idB),B);


% Formation initial pose
NSBF.pPos.Xr = [P.pPos.X(1:3); B.pPos.X(1:3)];

tp = tic;
tc = tic;
t  = tic;
ti = tic;


%% Send Comand do Take off 
disp(" ");
disp("Take off Command....")
disp(" ");
B.rTakeOff;
pause(5);


disp('Inicialize Test ..............');
disp('');


try
% Loop while error > erroMax
    while toc(t)< size(Qd,1)*(time)  %abs(NSB.pPos.Qtil(1))>erroMax(1) || abs(NSB.pPos.Qtil(2))>erroMax(2) || ...
%         abs(NSB.pPos.Qtil(4))>erroMax(4)|| abs(NSB.pPos.Qtil(5))>erroMax(5) ...
%         || abs(NSB.pPos.Qtil(6))>erroMax(6) % formation errors
    
    if toc(tc) > sampleTime                   

        t_atual = toc(t); 
        B.pPar.ti = tc;
        P.pPar.ti = tc;    
        P.pPar.Ts = toc(tc);
        tc = tic;  
        
        % Formation Error
        NSBF.pPos.Qtil = NSBF.pPos.Qd - NSBF.pPos.Q;
%         disp(NSBF.pPos.Qtil');

        %% Acquire sensors data
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>2
            P.pSC.U  = Rede.pMSG.getFrom{2}(29:30);  % current velocities (robot sensors)
            P1X       = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
        end
        
                
%         P.rGetSensorData;    % Pioneer
        % Get optitrack data
        rb = OPT.RigidBody;             % read optitrack
        
        % Pioneer
        P = getOptData(rb(idP),P);
        
        % Bebop        
%       B.rGetSensorDataOpt;             
        B = getOptData(rb(idB),B);        
        B.pSC.U = [B.pPos.X(4);B.pPos.X(5);B.pPos.X(9);B.pPos.X(12)]; % populates actual control signal to save data   

        % Formation Members Position        
        NSBF.mSetPose(P,B); 
        
        % Get Sample time to integrate
        NSBF.SampleTime = toc(ti);
        ti = tic;                                  % Reset Timer
%         disp(strcat("SampleTime: ",toStringJSON(NSBF.SampleTime)));
                
        %% Formation Control
%         NSBF.mFormationControl;                 % Formação Convencional 
        NSBF.mFormationPriorityControl;
%       NSBF.mPositionPriorityControl;
                     
        % Desired position ...........................................
%         NSBF.mInvTrans;
        
        % Envia Referencia para Pioneer
        P.pPos.Xd(1:3) = NSBF.pPos.Xr(1:3);         % desired position
        P.pPos.Xd(7:9) = NSBF.pPos.dXr(1:3);        % desired speed
        P.pPos.Xr(7:9) = NSBF.pPos.dXr(1:3);        % desired speed           
       
         
        % Envia Referencia para Drone
        B.pPos.Xda = B.pPos.Xd;    % save previous posture
        B.pPos.Xd(1:3) = NSBF.pPos.Xr(4:6);
        B.pPos.Xd(7:9) = NSBF.pPos.dXr(4:6); 
        B.pPar.Xr(7:9) = NSBF.pPos.dXr(4:6);        
        
%       Controlador do Bebop Dinamica Inversa Desacoplada  
        B.pSC.Kinematics_control = 1;        
        B.cInverseDynamicController_Compensador;
        
%       Controlador do Bebop Dinamica Inversa Conjunta           
%         B.cInverseDynamicController;
        
        % Send control signals to robots
        B = J.mControl(B);                      % joystick command (priority)                
        B.rCommand;
               
        % Pioneer
%         P.pSC.Kinematics_control = 1;
%         P = fDynamicController(P,cGains);     % Pioneer Dynamic Compensator                
%         P = fDynamicController(P);     % Pioneer Dynamic Compensator                
        P = fControladorCinematico(P,pgains);        
%         P = fControladorCinematico(P);
%         disp(P.pSC.Ud);
        Rede.mSendMsg(P);    
        
        % Get Performace Scores       
        NSBF.mFormationPerformace(sampleTime,toc(t),index);                                      
        
        % Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            B.pPos.Xd' B.pPos.X' B.pSC.Ud' B.pSC.U' NSBF.pPos.Qd' NSBF.pPos.Qtil' toc(t)]);
        fprintf(Arq,'\n\r');
        
        % Variable to feed plotResults function
        classArray = [classArray; P B];
        
        data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
                B.pPos.Xd' B.pPos.X' B.pSC.Ud' B.pSC.U' NSBF.pPos.Qd' NSBF.pPos.Qtil' toc(t)];
        
        comandos = [comandos; B.pSC.Ud(1) B.pSC.Ud(2) B.pSC.Ud(3) B.pSC.Ud(6)];
        poses = [poses; B.pPos.X(1) B.pPos.X(2) B.pPos.X(3) B.pPos.X(6)];
        velocidades = [velocidades; B.pPos.X(7) B.pPos.X(8) B.pPos.X(9) B.pPos.X(12)];
        dists = [dists; dist];
        Bebop = [Bebop;B];
        xds = [xds xd];
        xdps = [xdps xdp];

        tempo = [tempo toc(t)];
        
        % Increment Counter
        index = index + 1;   
                
    end
    
    
    % Draw robot
    if toc(tp) > inf
    %if toc(tp) > 0.5
        tp = tic;
        try
            delete(fig1);
            delete(fig2);
        catch
        end
        P.mCADdel                      % Pioneer
        P.mCADplot(1,'k');             % Pioneer modelo bonitão
        B.mCADplot;                    % ArDrone
        
        % Percourse made
        fig1 = plot3(data(:,13),data(:,14),data(:,15),'r-','LineWidth',0.8); hold on;
        fig2 = plot3(data(:,41),data(:,42),data(:,43),'b-','LineWidth',0.8);
        
%       view(75,30)
        axis([-6 6 -6 6 0 4]);
        grid on
        drawnow
        view(-21,30)
    end
    
    % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
    if btnEmergencia ~= 0 || B.pFlag.EmergencyStop ~= 0 || B.pFlag.isTracked ~= 1 ||...
       rb(idP).isTracked ~= 1 || rb(idB).isTracked ~= 1 
        disp('Bebop Landing through Emergency Command ');
        B.rCmdStop;
        B.rLand;

%         P.rCmdStop;
        P.pSC.Ud = [0  ;  0];
        Rede.mSendMsg(P);    

        break;
    end  
        
end
catch Ex
    disp(' ############## Bebop Landing through Try/Catch Loop Command #################');
    disp(Ex);
    
    B.rCmdStop;    
    B.rLand
    
%     P.rCmdStop;
    P.pSC.Ud = [0  ;  0];
    Rede.mSendMsg(P);     
end

% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    disp("End Land Command");
    B.rCmdStop;    
    B.rLand
    
%     P.rCmdStop;
    P.pSC.Ud = [0  ;  0];
    Rede.mSendMsg(P);    

    pause(dLandMsg);
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;
disp("Ros Shutdown completed...");

% NSB_PlotResults(data,1," - NSB - Formation Priority");
% NSBF.mDisplayFormationPerformace(" - NSB - Formation Priority");




%%
% figure
% hold on
% plot(tempo,poses(:,1))
% plot(tempo,poses(:,2))
% plot(tempo,poses(:,3))
% plot(tempo,poses(:,4))
% title('Posicoes')
% legend('Pos X','Pos Y','Pos Z', 'Ori Z');
% xlabel('Tempo(s)');
% ylabel('Pos');
% hold off
% 
% figure
% hold on
% plot(tempo,velocidades(:,1))
% plot(tempo,velocidades(:,2))
% plot(tempo,velocidades(:,3))
% plot(tempo,velocidades(:,4))
% title('Velocidades')
% xlabel('Tempo(s)');
% ylabel('Vel');
% 
% legend('Vel X','Vel Y','Vel Z', 'Vel Ang Z');
% hold off
% 
% figure
% hold on
% plot(tempo,comandos(:,1))
% plot(tempo,comandos(:,2))
% plot(tempo,comandos(:,3))
% title('Comandos')
% xlabel('Tempo(s)');
% ylabel('U');
% legend('U X','U Y','U Z');
% hold off
% 
% figure
% hold on
% plot(tempo,dists)
% title('Dist')
% hold off
% 
% 
% figure
% hold on
% plot(tempo,comandosP(:,1))
% plot(tempo,comandosP(:,2))
% title('Comandos Pioneer')
% xlabel('Tempo(s)');
% ylabel('U');
% legend('U X','U Y');
% hold off

% data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
%         B.pPos.Xd' B.pPos.X' B.pSC.Ud' B.pSC.U' NSBF.pPos.Qd' NSBF.pPos.Qtil' toc(t)];
        


% Erro Posição Formação
Xtil = data(:,end-6:end-1);
figure();
hold on;
    grid on;
    plot(data(:,end),Xtil(:,1));
    plot(data(:,end),Xtil(:,2));
    plot(data(:,end),Xtil(:,3));
    plot(data(:,end),Xtil(:,4));
    plot(data(:,end),Xtil(:,5));
    plot(data(:,end),Xtil(:,6));
    title('Erro de Fromação');
    legend('Pos X','Pos Y','Pos Z', 'Rho', 'Apha', 'Beta');
    xlabel('Tempo[s]');
    ylabel('Erro [m]');
hold off;


% Erro Posição Pioneer
Xtil = data(:,1:12) - data(:,13:24);
figure();
hold on;
    grid on;
    plot(data(:,end),Xtil(:,1));
    plot(data(:,end),Xtil(:,2));
    plot(data(:,end),Xtil(:,3));
    title('Erro de Pioneer');
    legend('Pos X','Pos Y','Pos Z');
    xlabel('Tempo[s]');
    ylabel('Erro [m]');
hold off;


% Erro Posição Drone
Xtil = data(:,29:40) - data(:,41:52);
figure();
hold on;
    grid on;
    plot(data(:,end),Xtil(:,1));
    plot(data(:,end),Xtil(:,2));
    plot(data(:,end),Xtil(:,3));
    title('Erro de Drone');
    legend('Pos X','Pos Y','Pos Z');
    xlabel('Tempo[s]');
    ylabel('Erro [m]');
hold off;



disp("Figures Printed...");
 
% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx




