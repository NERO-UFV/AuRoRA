%% Pioneer Position Control

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  %
% Initial Comands

clear; close all; clc;

try
    fclose(instrfindall);
catch
end
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %
% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Classes

% Robots
P = Pioneer3DX(1);

% Joystick
J = JoyControl;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Network
Rede = NetDataShare;

%% Network communication check
tm = tic;
while true
    
    if isempty(Rede.pMSG.getFrom)
        Rede.mSendMsg(P);Rede.mSendMsg(P);Rede.mSendMsg(P);
        if toc(tm) > 1/30
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        end
    elseif length(Rede.pMSG.getFrom) > 1
        if isempty(Rede.pMSG.getFrom{2})
            Rede.mSendMsg(P);
            
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message 1......')
            
        else
            break
        end
    end
end

%% Robots initial pose

idP = getID(OPT,P);            % pioneer ID on optitrack
rb = OPT.RigidBody;            % read optitrack data
P = getOptData(rb(idP),P);    % get pioneer data


%% Variable initialization
data = [];

%% Simulation

fprintf('\nStart..............\n\n');

% Time variables initialization

T_CONTROL = 1/30;
T_ARDRONE = 1/30;

P.pPar.Ts = 1/30;

rX = 1.75;      % [m]
rY = 1.75;      % [m]
rho = 1.5;      % [m]
T = 60;         % [s]
Tf = 120;       % [s]
w = 2*pi/T;     % [rad/s]

T1 = 120.0;             % Lemniscata
% % T2 =  15.0 + T1;        % Aproximação + Emergency
% % T3 =  0.1 + T2;         % Andando com o Drone pousado
% % T4 =  5.0 + T3;         % Parando o Pioneer

t_control = tic;
t_integ_1 = tic;
t  = tic;

while toc(t) < T1
    
    if toc(t_control) > T_CONTROL
        %% Get data from robot and Optitrack
        t_control = tic;
        Rede.mReceiveMsg;
        
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{2}(29:30);       % current velocities (robot sensors)
            PX       = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
        end
        
        rb = OPT.RigidBody;             % read optitrack
        P = getOptData(rb(idP),P);
        
       %% Trajectory
        
        t_traj = toc(t);
        a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
        tp = a*Tf;

        % Pioneer
        P.pPos.Xda = P.pPos.Xd;    % save previous posture
        
        P.pPos.Xd(1:3) = [ rX*sin(w*tp); rY*sin(2*w*tp); 0 ];
        P.pPos.Xd(7:9) = [ w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp); ...
                           2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp);...
                           0];
        
        %% Control
        
%         cgains = [ 0.3  0.3  0.75  0.75  0.75  0.75  0.10  0.03 ];

        cgains = [ 0.3  0.3  0.75  0.75  0.75  0.75  0.10  0.03 ];

        P = fDynamicController(P);     % Pioneer Dynamic Compensator

        %% Save data
        % Variable to feed plotResults function
        data = [  data  ; P.pPos.Xd'     P.pPos.X'        P.pSC.Ud(1:2)'    P.pSC.U(1:2)' toc(t)];
        
        % %       %   1 -- 12         13 -- 24          25 -- 26           27 -- 28
        % %       %   P1.pPos.Xd'     P1.pPos.X'        P1.pSC.Ud'         P1.pSC.U'

        %% Send control signals to robots

        Rede.mSendMsg(P);

        %         if toc(t_control) > 0.030
        %             disp(['Estourou o tempo: ',num2str(1000*(toc(t_control)-0.030))]);
        %         end
        
    end
    
end

%% Send control signals
P.pSC.Ud = -[.1  ;  0];

for ii = 1:50
    Rede.mSendMsg(P);
end

%% Send control signals
P.pSC.Ud = [0  ;  0];
for ii = 1:50
    Rede.mSendMsg(P);
end

%% Plot results

% Drone data
PXd   = data(:,(1:12))';
PX    = data(:,12+(1:12))';
PUd   = data(:,24+(1:2))';
PU    = data(:,26+(1:2))';
PXtil = PXd - PX;

sl= 1; %'default';    % largura da linha
st = 14;  % tamanho da fonte
ss = 2;   % tamanho dos símbolos

% Posição dos robôs
% Parameters
scale  = 1;     % robot model plot scale
Pcolor = 'k';   % robot A color
step   = 200;   % model plot step
h      = 0.1;   % line height

figure;
axis equal
axis ([-2 2 -2 2 0 2])
set(gca,'Box','on')

hold on, grid on;

% Initial positions
ps1 = plot3(PXd(1,1),PXd(2,1),PXd(3,1),'r^','MarkerSize',5,'LineWidth',3);
ps2 = plot3(PX(1,1),PX(2,1),PX(3,1),'c^','MarkerSize',5,'LineWidth',3);


% % Percourse made
p1 = plot3(PXd(1,:),PXd(2,:),PXd(3,:),'r-','LineWidth',0.5);
p2 = plot3(PX(1,:),PX(2,:),PX(3,:),'b-','LineWidth',0.5);

xlabel('$x$ [m]','interpreter','Latex');
ylabel('$y$ [m]','Interpreter','latex');
zlabel('$z$ [m]','interpreter','Latex');



figure();
subplot(211)
hold on, grid on;
axis ([0 data(end,29) 0 1])
plot(data(:,29),PUd(1,:),'r-','LineWidth',0.5);
plot(data(:,29),PU(1,:),'b-','LineWidth',0.5);

subplot(212)
hold on, grid on;
axis ([0 data(end,29) -1 1])
plot(data(:,29),PUd(2,:),'r-','LineWidth',0.5);
plot(data(:,29),PU(2,:),'b-','LineWidth',0.5);




% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
