close all
clear
clc

%% Reagir ao ambiente:
% d = sqrt(X(:,1).^2 +X(:,2).^2);
% if min(d(120:270)) < .5
%     disp('parar pioneer')
% else
%     disp('executar desvio')
%     
% end
%%

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
P.pPar.a = 0;
P.pPar.alpha = (0)*pi/180;


% P = Pioneer3DX;
% P.rConnect;
% P.rSetPose([0 0 0 0]);
% P.mCADmake;

subLaser = rossubscriber('/scan','sensor_msgs/LaserScan');
Laser = [];
%%
for ii = 1:100
    Laser = subLaser.LatestMessage;
end
% LaserData = Laser.readCartesian;

% Parâmetros de Navegação
DOBS_TAN  = 0.85;
DOBS_A    = 0.65;
DOBS_TRAN = 1;
FLAG_DESVIO = false;

% Parâmetros Sonsar
% maxLaserDist = 20.0; % [m]
% 
% distRobotFrame = [(-180:180)*pi/180; ones(1,361)*maxLaserDist];
% for ii = 1:size(distRobotFrame,2)
%     distWorldFrame(:,ii) = P.pPos.X(1:2) + distRobotFrame(2,ii)*...
%         [cos(P.pPos.X(6)) -sin(P.pPos.X(6)); sin(P.pPos.X(6)) cos(P.pPos.X(6))]*...
%         [cos(distRobotFrame(1,ii));sin(distRobotFrame(1,ii))];
% end

t_curve = 10;
desvio = 0;
t = [];
t_amostragem = P.pPar.Ts;
t_plot = 0.1;
ta = tic;
tp = tic;
d = tic;

figure
P.mCADdel;
P.mCADplot2D('r');
axis([-6 6 -6 6])
hold on
grid on
drawnow;

%%% codigo para armazenar dados de navegação %%%

P.rEnableMotors;
P_Obs = [];
Mundo = [];
dist_min = inf;
dist_seguranca = 0.2500;

while J.pFlag == 0 % && DIST_MIN > 0.7 + DIST_SEGURANCA
    if toc(ta) > t_amostragem
        ta = tic;

%         P.rGetSensorData;
        rb = OPT.RigidBody;             % read optitrack
        
        try
            if rb(idP).isTracked
                P = getOptData(rb(idP),P);
%                 disp(P.pPos.X)
            end
        catch
        end
        %LASER
        Laser = subLaser.LatestMessage;
        LaserD = Laser.readCartesian;
        LaserData = LaserD(5:end-5,:);
%         if false
        LaserNorm = [];
        for ii = 4:size(LaserData,1)
            LaserNorm(ii,:) = norm(LaserData(ii-1,:))-norm(LaserData(ii,:));
        end
        nNyq = 1;
        for ii = (nNyq+100):nNyq:(size(LaserData,1)-100)
%         for ii = (nNyq):nNyq:(size(LaserData,1))
            LaserDCorte((ii-100)/nNyq,:) = LaserData(ii,:);
        end
        for ii = 2:size(LaserDCorte,1)
            LaserNCorte(ii,:) = norm(LaserDCorte(ii-1,:))-norm(LaserDCorte(ii,:));
        end
%         plot(LaserDCorte(:,1),LaserDCorte(:,2),'.b')
%         figure
%         stem(1:size(LaserDCorte,1),LaserNCorte)
        
        
        Limiar = 0.2*max(LaserNCorte);
        
        P_Limiar = find(abs(LaserNCorte)>Limiar);
        dist_min = min(sqrt(LaserDCorte(P_Limiar(1):P_Limiar(2),1).^2+LaserDCorte(P_Limiar(1):P_Limiar(2),2).^2));
%         DIST_MIN = min(sqrt(LaserDCorte(:,1).^2+LaserDCorte(:,2).^2));    
        P_Obs = find((P_Limiar(2:end)-P_Limiar(1:end-1))>2);
                      
        if dist_min < 0.55 + dist_seguranca || desvio == 1
            if desvio == 0
                psi = P.pPos.X(6);
                % Rotação em Z
                Rz = [cos(psi) -sin(psi);
                      sin(psi) cos(psi)];

                % Rotação em Z
                RNz = [cos(-pi/2) -sin(-pi/2);
                      sin(-pi/2) cos(-pi/2)];

                LaserReal = P.pPos.X(1:2).*ones(2,size(LaserDCorte,1)) + Rz*LaserDCorte';
                VD_Obs = LaserReal(:,P_Limiar(2)-1)-LaserReal(:,P_Limiar(1));
                N_Obs = RNz*VD_Obs;
                Bx = [P.pPos.X(1) P.pPos.X(1)+VD_Obs(1) P.pPos.X(1)+VD_Obs(1)+N_Obs(1)]';
                By = [P.pPos.X(2) P.pPos.X(2)+VD_Obs(2) P.pPos.X(2)+VD_Obs(2)+N_Obs(2)]';
                sympref('HeavisideAtOrigin',1)
                syms bc
                B = bernsteinMatrix(2,bc);
                bezierCurve = simplify(B*[Bx By]);
                f(bc) = [bezierCurve];      
%                     if toc(tp) < t_amostragem
%                         P.pPos.Xd(1:2) = [0.4918;1.6497]; 
%                     else 
%                         t = tic;
%                         P.pPos.Xd(1:2) = [0.4918;1.6497];
%                     end         
            end
            desvio = 1;
            if isempty(t)
                t = tic;
            end
            XdA = Xd';
            Xd = vpa(f(toc(t)/t_curve));
            P.pPos.Xd(1:2) = [Xd(1); Xd(2)];
            P.pPos.Xd(7:8) = (P.pPos.Xd(1:2) - XdA)/toc(d);
            d = tic;
            if toc(t) > t_curve
%                 P.pPos.Xd(1:2) = [0.4918;1.6497];
                J.pFlag = 1;
            end
        else            
%             Xd = [1.5 1.5];
            P.pPos.Xd(1:2) = [1.3778;1.6497]; 
            rx = 1;
            ry = 1;
            if toc(ta) > t_amostragem 
                ta = tic;
                XdA = Xd;
            %    Lemniscata (8')
            %     Xd = [rx*sin(2*W*toc(T)); ry*sin(W*toc(T)); 0];

            %    Circunferencia
                Xd = [rx*cos(W*toc(t)); ry*sin(W*toc(t)); 0];

            %    Posição
            %     Xd = [1; 1.5; 0];

                dXd = (Xd-XdA)/toc(d);
                d = tic;

                % Acquire sensors data        
                % Get optitrack data        
                rb = OPT.RigidBody;             % read optitrack

                % Pioneer
                P = getOptData(rb(idP),P);

                P.pPos.Xd(1:3) = Xd;
                P.pPos.Xd(7:9) = dXd;
% 
% 
                P = fKinematicControllerExtended(P);      
% 
                P = J.mControl(P);
% 
                P.rCommand;
% 
            end
        end 
%         
%         P.pPos.Xd(1:2) = [1.5;1.5]; 
        
        % Controle
        P = fKinematicControllerExtended(P);

        P = J.mControl(P);
        
        P.rCommand;
%         P.pSC.Ud = [0;0];
        
    end
    
    if toc(tp) > t_plot
        tp = tic;
        try
            P.mCADdel;
            delete(H)
        end
        
        psi = P.pPos.X(6);
        % Rotação em Z
        Rz = [cos(psi) -sin(psi);
              sin(psi) cos(psi)];
        
        LaserPlot = P.pPos.X(1:2).*ones(2,size(LaserDCorte,1)) + Rz*LaserDCorte';  
          
        P.mCADplot2D('r');
        H = plot(LaserPlot(1,:),LaserPlot(2,:),'.b');
        
        drawnow
    end
end

% P.pSC.Ud = [0;0];
P.rCommand;

J.pFlag = 0;
P.rDisableMotors;