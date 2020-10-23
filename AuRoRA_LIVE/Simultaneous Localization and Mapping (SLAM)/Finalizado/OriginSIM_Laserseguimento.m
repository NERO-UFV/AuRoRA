close all
clear
clc

try
    fclose(instrfindall);
end

try
    % Load Classes
    
    RI = RosInterface;
    setenv('ROS_IP','192.168.0.158')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    
%     % Inicializando o OptiTrack
    OPT = OptiTrack;    % Criando o OptiTrack
    OPT.Initialize;     % Iniciando o OptiTrack
    
    P = RPioneer(1,'P1'); % Pìoneer3DX Experimento
    idP = 1;
    P.rDisableMotors;
    
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

% P = Pioneer3DX;
% P.rConnect;
% P.rSetPose([2.5 2.5 0 0]);

subLaser = rossubscriber('/scan','sensor_msgs/LaserScan');
Laser = [];
for ii = 1:100
    Laser = subLaser.LatestMessage;
end
LaserData = Laser.readCartesian;

% Parâmetros de Navegação
DOBS_TAN  = 0.85;
DOBS_A    = 0.65;
DOBS_TRAN = 1;
FLAG_DESVIO = false;

% Parâmetros Sonar
maxLaserDist = 5.0; % [m]

Medidas = ones(1,361)*maxLaserDist;

% Criar grade de ocupação
og = robotics.OccupancyGrid(25,25,10);
og.GridLocationInWorld = [-5 -5];


% Parâmetros Desvio Tangencial
sTE = TangentialEscape;
dref = 0.50;
k1 = [0.25 0;0 0.35];
k2 = [1 0;0 1];
xq = []; yq = []; grid = []; curva = []; poly = [];

prog = true;
pos_dest = [18 18;2.5 2.5];
pos_index = 1;
%%
%%% codigo para armazenar dados de navegação %%%

P.rEnableMotors;
Mundo = [];
while pos_index <= size(pos_dest,1)
    Xdest = pos_dest(pos_index,1); Ydest = pos_dest(pos_index,2);
    P.pPos.Xd([1 2 6]) = [Xdest Ydest 0]';
    d = 0;
    ts = tic;
    P.pPos.Xtil(1:2) = [inf,inf];
    mov_aestrela = [];
    grid1 = [];
    grid1.og = og;
    grid1.mod = floor(og.occupancyMatrix+0.5);
    [xq,yq,mov_aestrela,tem_caminho,grid,curva] = ProcuraCaminho(world2grid(og,[P.pPos.X(1) P.pPos.X(2)]),world2grid(og,[P.pPos.Xd(1) P.pPos.Xd(2)]),grid1);
    if tem_caminho
        dobs = DOBS_A;
        rota = ConstruirCaminhoSeguimento([xq yq]);
        t_rota = size(rota,2);
    else
        dobs = DOBS_TAN;
    end
    
    while norm([Xdest-P.pPos.X(1) Ydest-P.pPos.X(2)]) > 0.15
       % P.rGetSensorData;
       % LaserData = P.rGetSonarData;
        rb = OPT.RigidBody;             % read optitrack
        
        try
            if rb(idP).isTracked
                P = getOptData(rb(idP),P);
%                 disp(P.pPos.X)
            end
%         insertRay(og,[P.pPos.X(1) P.pPos.X(2) P.pPos.X(6)],LaserData(2,:),LaserData(1,:),maxLaserDist);
        if tem_caminho
            if norm(P.pPos.X(1:2)-[rota(1,t_rota) rota(2,t_rota)]') < 0.1
                tem_caminho = false;
                dobs = DOBS_TAN;
                P.pPos.Xd(1:2) = [Xdest Ydest];
                fprintf('parar de seguir caminho\n');
            else
                dtemp = min(LaserData.medidas(2,:));
                if dtemp <= DOBS_A
                    FLAG_DESVIO = true;
                    fprintf('iniciando desvio apesar do A* \n');
                end
                if FLAG_DESVIO && dtemp < DOBS_TRAN
                    P.pPos.Xd(1:2) = [Xdest Ydest];
                    fprintf('continuar desviando...\n');
                else
                    FLAG_DESVIO = false;
                    P.pPos.Xd = SeguirCaminho(rota,P.pPos.X);
                end
            end
        end
        
        % Desvio Tangencial
        P = sTE.sFindVirtualGoal(P,LaserData);
        P = fControladorCinematico(P);
        
        P = J.mControl(P);
        
        if toc(ts) > 0.1
            P.rCommand;
            ts = tic;
        end
        
        show(og)
        drawnow;
    end
    P.pSC.Ud = [0; 0];
    P.rSendControlSignals;
    pos_index=pos_index+1;
    %%% codigo para salvar dados de navegação %%%
    %POS_X{pos_index} = rastroRobo;
    %rastroRobo = [];
    %if tem_caminho
    %    POS_A_ESTRELA{pos_index} = [xq yq];
    %end
    %M_SENSOR{pos_index} = rastroSensor;
end

P.pSC.Ud = [0; 0];
