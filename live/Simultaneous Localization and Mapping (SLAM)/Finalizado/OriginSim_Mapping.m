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
% P.rSetPose([0 0 0 0]);
% P.mCADmake;

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

% Parâmetros Sonsar
maxLaserDist = 20.0; % [m]

distRobotFrame = [(-180:180)*pi/180; ones(1,361)*maxLaserDist];
for ii = 1:size(distRobotFrame,2)
    distWorldFrame(:,ii) = P.pPos.X(1:2) + distRobotFrame(2,ii)*...
        [cos(P.pPos.X(6)) -sin(P.pPos.X(6)); sin(P.pPos.X(6)) cos(P.pPos.X(6))]*...
        [cos(distRobotFrame(1,ii));sin(distRobotFrame(1,ii))];
end

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
pos_dest = [3 5; 0 0; 3 5; 0 0];
pos_index = 1;

figure
P.mCADdel;
P.mCADplot2D('r');
axis([-5 8 -5 8])
hold on
drawnow;
%%
%%% codigo para armazenar dados de navegação %%%

P.rEnableMotors;
Mundo = [];
while pos_index <= size(pos_dest,1) && J.pFlag == 0
    Xdest = pos_dest(pos_index,1); Ydest = pos_dest(pos_index,2);
    
    d = 0;
    ts = tic;
    P.pPos.Xtil(1:2) = [inf,inf];
    
    while norm([Xdest-P.pPos.X(1) Ydest-P.pPos.X(2)]) > 0.15 && J.pFlag == 0
        P.pPos.Xd([1 2 6]) = [Xdest Ydest 0]';        

%         P.rGetSensorData;
        rb = OPT.RigidBody;             % read optitrack
        
        try
            if rb(idP).isTracked
                P = getOptData(rb(idP),P);
                disp(P.pPos.X)
            end
        catch
        end
        %LASER
        Laser = subLaser.LatestMessage;
        LaserData = Laser.readCartesian;
        
        %insertRay(og,[P.pPos.X(1) P.pPos.X(2) P.pPos.X(6)],LaserData(2,:),LaserData(1,:),maxLaserDist);
        
        % Construir sensor virtual
        % Recalcular posição dos objetos no mundo
        try 
            delete(m1)
            delete(m2)
            delete(m3)
        end
%         m1 = plot(distWorldFrame(1,:),distWorldFrame(2,:),'.k');
        
        for ii = 1:size(distRobotFrame,2)
            theta = atan2(distWorldFrame(2,ii)-P.pPos.X(2),distWorldFrame(1,ii)-P.pPos.X(1));
            beta  = theta - P.pPos.X(6);
            if abs(beta) > pi
                if beta > 0
                    beta = beta - 2*pi;
                else
                    beta = beta + 2*pi;
                end
            end
            rho = norm(P.pPos.X(1:2)-distWorldFrame(:,ii));
            if rho < 0.95*maxLaserDist
                distRobotFrame(2,181+floor(rad2deg(beta))) = rho;
            else
                distRobotFrame(2,181+round(rad2deg(beta))) = maxLaserDist;
            end
        end
        
        % Atualizar com os valores atuais do Sonar
%         distRobotFrame(2,181+round(rad2deg(LaserData(1,:)))) = LaserData(2,:);
        
        % Posicionar medidas atuais no mundo
        for ii = 1:size(distRobotFrame,2)
            distWorldFrame(:,ii) = P.pPos.X(1:2) + distRobotFrame(2,ii)*...
                [cos(P.pPos.X(6)) -sin(P.pPos.X(6)); sin(P.pPos.X(6)) cos(P.pPos.X(6))]*...
                [cos(distRobotFrame(1,ii));sin(distRobotFrame(1,ii))];
            
%             if distRobotFrame(2,ii) < 0.95*maxSonarDist
%                 Mundo = [Mundo distWorldFrame(:,ii)];
%             end
        end
%         m2 = plot(distWorldFrame(1,:),distWorldFrame(2,:),'.r');
        
        % Desvio Tangencial
%         P = sTE.sFindVirtualGoal(P,distRobotFrame(:,91:271));
        
        P = sTE.sFindVirtualGoal(P,LaserData);
        P.pPos.Xd(1:2) = [0;1];
        P = fControladorCinematico(P);
        
        P = J.mControl(P);
        
        if toc(ts) > 0.1
            P.rCommand;
            ts = tic;
        end
        
        % show(og)
        try 
            delete(h1)
            delete(h2)
            delete(h3)
        end
%         m3 = plot(distWorldFrame(1,:),distWorldFrame(2,:),'-b');
        
%         for ii = 1:8
%             h1(ii) = plot(P.pPos.X(1)+[0 LaserData(2,ii).*cos(LaserData(1,ii)+P.pPos.X(6))],...
%                 P.pPos.X(2)+[0 LaserData(2,ii).*sin(LaserData(1,ii)+P.pPos.X(6))],'-*k');
%         end
%           
%         h2 = plot([P.pPos.X(1) P.pPos.Xd(1)],[P.pPos.X(2) P.pPos.Xd(2)],'-og');
%         try
%             h3 = plot(sTE.pList(1,:),sTE.pList(2,:),'oc');
%         end
%         P.mCADdel;
%         P.mCADplot2D('r');
%         axis([-5 10 -5 10])
%         drawnow;
    end
    P.pSC.Ud = [0; 0];
    P.rCommand;
    pos_index=pos_index+1;
    %%% codigo para salvar dados de navegação %%%
    %POS_X{pos_index} = rastroRobo;
    %rastroRobo = [];
    %if tem_caminho
    %    POS_A_ESTRELA{pos_index} = [xq yq];
    %end
    %M_SENSOR{pos_index} = rastroSensor;
end

%%
P.rDisableMotors;