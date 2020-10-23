%% 3D Line Formation Pioneer-Drone
% Position task using a 3D line virtual structure to control the formation
%
%  In this script, the ArDrone is commanded to land during the
%  experiment. Later, its take off again to continue the task until the
%  end.
%
% I.   Pioneer is the reference of the formation
% II.  The formation variables are:
%      Q = [xf yf zf rhof alfaf betaf]
%  *the formation variables are specified in formation class file (LineFormation3D.m)

clear; close all; clc;

try
    fclose(instrfindall);
catch
end

%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)));
addpath(genpath(pwd));

%% Load Classes
% Robot
P = Pioneer3DX;

% % % % % % P.rConnect;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Network
Rede = NetDataShare;

pause(2);

%% Network communication check
tm = tic;
while true
    
    if isempty(Rede.pMSG.getFrom)
        Rede.mSendMsg(P);
        if toc(tm) > 0.1
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        end
    elseif length(Rede.pMSG.getFrom) > 1
        if isempty(Rede.pMSG.getFrom{2})
            Rede.mSendMsg(P);
            
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
            
        else
            break
        end
    end
end
clc
disp('Data received. Continuing program...');

 %% Robots initial pose
 
% detect rigid body ID from optitrack
idP = getID(OPT,P);          % pioneer ID on optitrack
rb = OPT.RigidBody;          % read optitrack data
P = getOptData(rb(idP),P);   % get pioneer data
Rede.mSendMsg(P)

%% Variable initialization

% Desired positions vector [xf yf ]

Xd = [   0.00   0.00    ;
         1.25   1.25    ;
        -1.25   1.25    ;
        -1.25  -1.25    ;
         1.25  -1.25   ];

cont = 0;     % counter to change desired position through simulation
tchange = 20;    % time to change desired positions [s]

Hist = [];

%% Plot inicial
figure(1);
axis([-4 4 -4 4]);
xlabel('Eixo x [m]','FontSize',12,'FontWeight','bold');
ylabel('Eixo y [m]','FontSize',12,'FontWeight','bold');
hold on;
grid on;

P.mCADdel;
P.mCADplot(.7,'r');
p1 = plot(0,0,'r--','LineWidth',2);
p2 = plot(0,0,'b','LineWidth',2);
legend([p1,p2],'X_{d}','X','Location','northwest');
drawnow;

pause(2);

%% Simulation

pos = 0;
traj = 1;

% Time variables initialization
tsim    = size(Xd,1)*(tchange);
tsim = 90;
t       = tic;
tc      = tic;
tplot      = tic;

while toc(t)< tsim
    
    %% Desired positions
    if toc(tc) > 1/30
        tc = tic;
        
        %% Change positions
        
        if pos == 1 && traj == 0
            if toc(t)> cont*tchange
                cont = cont + 1;
            end
            if cont <= size(Xd,1)
                P.pPos.Xd(1:2) = Xd(cont,:)';
            end
            
        elseif traj == 1 && pos == 0
            
            rA = 1.00; % [m]
            rB = 1.00; % [m]
            T = 90; % [s]
            w = 2*pi/T; % [rad/s]
            
            
            P.pPos.Xd(1) = rA*sin(w*toc(t));
            P.pPos.Xd(2) = rB*sin(2*w*toc(t));
            P.pPos.Xd(7) = rA*w*cos(w*toc(t));
            P.pPos.Xd(8) = 2*rB*w*cos(2*w*toc(t));
        end
        
        %% Acquire sensors data
        % Get network data
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{2}(29:30);  % current velocities (robot sensors)
            PX       = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
        end
        

%         P.pPos.X = PX;
        % Get optitrack data
        rb = OPT.RigidBody;             % read optitrack
        
        % Pioneer
        P = getOptData(rb(idP),P);
        
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
% % % %         P.rGetSensorData;
        
        K = [ cos(P.pPos.X(6))   -P.pPar.a*sin(P.pPos.X(6)); ...
            sin(P.pPos.X(6))    P.pPar.a*cos(P.pPos.X(6)) ];
        
        %% Controlador Cinemático
        
        % Tanh
        
        k1 = [0.50 0.00 ; 0.00 0.50]; % ganho de saturação
        k2 = [0.50 0.00 ; 0.00 0.50]; % ganho da inclinação da "reta"
        
        
% % %         k1 = [.5 0.00 ; 0.00 .5]; % ganho de saturação
% % %         k2 = [1.25 0.00 ; 0.00 1.25]; % ganho da inclinação da "reta"
        
                 
        U = k1*tanh(k2*P.pPos.Xtil(1:2)); % sinal de controle
        

        
% % %         P.pSC.Ud = K\(P.pPos.Xd([7, 8]) + U); % Sem compensador usar este
        
        %% Compensador dinâmico
        
        P.pSC.Ur = K\(P.pPos.Xd([7, 8]) + U); % Com compensador usar este
        P = fCompensadorDinamico(P);
        
        %         sInvKinematicModel(P,P.pPos.Xd(7:8));  % sem essa conversão o compensador não tem o valor de pSC.Ur  (por isso o pioneer estava ficando parado, Valentim)
            
%         P = fDynamicController(P); 
        
        %% Send control signals to robots
        Rede.mSendMsg(P);       % send Pioneer data to network
        
        Hist = [Hist; [P.pPos.Xd' P.pPos.X' P.pPos.Xtil' P.pSC.U' P.pSC.Ud' P.pSC.Ur' toc(t) PX']];
        
        %% Plot
        if toc(tplot) > 0.1
            
            tplot = tic;
            P.mCADdel;
            P.mCADplot(.7,'r');
            
            % Plotar rastros
            
            if pos == 0 && traj == 1
                fig(1) = plot(Hist(:,1),Hist(:,2),'r--','LineWidth',2);
                fig(2) = plot(Hist(:,13),Hist(:,14),'b','LineWidth',1);
                legend([p1,p2],'X_{d}','X','Location','northwest');
                
            else
                fig(1) = plot(Hist(:,1),Hist(:,2),'or','LineWidth',2);
                fig(2) = plot(Hist(:,13),Hist(:,14),'b','LineWidth',1);
                legend([p1,p2],'X_{d}','X','Location','northwest');
                
            end
            
            drawnow;
            
        end
    end
end

%% Tese de envio de Ud para o Pioneer

P.pSC.Ud = [0.1;0];
for ii = 1:5
    Rede.mSendMsg(P);
end

%%  Stop robot
% Send control signals
% Send to network (a few times to be sure)
P.pSC.Ud = [0;0];
for ii = 1:5
    Rede.mSendMsg(P);
end

%% Plot Final

    figure(2);
    plot(Hist(:,43),Hist(:,13),'k','LineWidth',2);
    hold on;
    plot(Hist(:,43),Hist(:,44),'--r','LineWidth',2);
    grid on;
    legend('X_{OPT}','X_{ODM}');
    axis([0 tsim -2 2]);
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    ylabel('Eixo x [m]','FontSize',12,'FontWeight','bold');
    
    figure(3);
    plot(Hist(:,43),Hist(:,14),'k','LineWidth',2);
    hold on;
    plot(Hist(:,43),Hist(:,45),'--r','LineWidth',2);
    grid on;
    legend('Y_{OPT}','Y_{ODM}');
    axis([0 tsim -2 2]);
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    ylabel('Eixo y [m]','FontSize',12,'FontWeight','bold');
    
    figure(4);
    plot(Hist(:,43),Hist(:,18),'k','LineWidth',2);
    hold on;
    plot(Hist(:,43),Hist(:,49),'--r','LineWidth',2);
    grid on;
    legend('Y_{OPT}','Y_{ODM}');
%     axis([0 tsim -2 2]);
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    ylabel('Eixo y [m]','FontSize',12,'FontWeight','bold');
    
        figure(5);
    plot(Hist(:,43),Hist(:,39),'--r','LineWidth',2);
    hold on;
    plot(Hist(:,43),Hist(:,37),'k','LineWidth',2);
    plot(Hist(:,43),.75*ones(length(Hist)),'--b','LineWidth',2);
    grid on;
    legend('U_{d}','U','U_{sat}');
%     axis([0 tmax 0 1]);
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    ylabel('Velocidade Linear [m/s]','FontSize',12,'FontWeight','bold');

    % Velocidade angular [ Ud(2) , U(2) ]
    figure(6);
    plot(Hist(:,43),180*Hist(:,40)/pi,'--r','LineWidth',2);
    hold on;
    plot(Hist(:,43),180*Hist(:,38)/pi,'k','LineWidth',2);
    plot(Hist(:,43),[100*ones(length(Hist)), -100*ones(length(Hist))],'--b','LineWidth',2);
    grid on;
    legend('U_{d}','U','U_{sat}');
%     axis([0 tmax -250 250]);
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    ylabel('Velocidade Angular [°/s]','FontSize',12,'FontWeight','bold');
