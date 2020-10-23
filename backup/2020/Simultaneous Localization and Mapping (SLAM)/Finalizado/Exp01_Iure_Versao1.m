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

t_amostragem = 1/10;
t_plot = 0.1;
ta = tic;
tp = tic;

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
while J.pFlag == 0 % && isempty(P_Obs)
    if toc(ta) > t_amostragem
        ta = tic;

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
        LaserD = Laser.readCartesian;
        LaserData = LaserD(5:end-5,:);
%         if false
        LaserNorm = [];
        for ii = 4:size(LaserData,1)
            LaserNorm(ii,:) = norm(LaserData(ii-1,:))-norm(LaserData(ii,:));
        end
        nNyq = 25;
        for ii = nNyq:nNyq:size(LaserData,1)
            LaserDCorte(ii/nNyq,:) = LaserData(ii,:);
        end
        for ii = 2:size(LaserDCorte,1)
            LaserNCorte(ii,:) = norm(LaserDCorte(ii-1,:))-norm(LaserDCorte(ii,:));
        end
%         plot(LaserDCorte(:,1),LaserDCorte(:,2),'.b')
%         figure
%         stem(1:size(LaserDCorte,1),LaserNCorte)
        
        
        Limiar = 0.05;
        
        P_Limiar = find(abs(LaserNCorte)>Limiar);
        P_Obs = find((P_Limiar(2:end)-P_Limiar(1:end-1))>2);
                      
        if ~isempty(P_Obs)
            % Variáveis para o caminho:
            
            %Definindo variaveis:
            xs  = 0.4589;                        ys = 0.9448;               %Posição inicial
            xf  = 0.95;                          yf = 1.8084;               %Posição final
            ths = 0;                        thf = (120)*(pi/180);   %Orientação inicial/final

            %Pontos de controle:
            b0 = [xs;ys];                   b5 = [xf;yf];   
            c0 = 2;                         c5 = 2;
            d0 = c0.*[cos(ths),sin(ths)];   d5 = c5.*[cos(thf),sin(thf)];

            b1 = b0 + [(1/5).*d0]';         b4 = b5 - [(1/5).*d5]';

            b2 = b1 + [(1/5).*d0]';         b3 = b4 - [(1/5).*d5]';       


            ControlPoints = [b0,b1,b2,b3,b4,b5]; %=> N = 6;
            %Plot da curva
            t = 0:0.01:1;
            N = 6;
            Sum = 0;    

            for i=0:N-1
                Sum = Sum + nchoosek(N-1,i)*(1-t).^(N-1-i).*(t.^i).*ControlPoints(:,i+1);
            end
            Curve = Sum;

            plot(xs,ys,'k*')
            hold on
            plot(xf,yf,'r*')
            hold on
            plot(ControlPoints(1,:),ControlPoints(2,:),'bo')
            hold on
            plot(ControlPoints(1,:),ControlPoints(2,:),'k*')
            hold on
            plot(Curve(1,:),Curve(2,:),'b--')
            hold on
            plot(Curve(1,:),Curve(2,:),'b*')
            axis([-1 4 -1 4])
            grid on


            %% Rotina da simulação:
            t=tic;  ta=tic;   tp = tic;         tc=tic; tcmax=35;
                    tmax=40;
            k1=0.6; k2=0.4;   it = 0;

            while toc(t)<tmax
                if toc(ta)>0.01
                    ta=tic;

                    P.pPos.Xda     = P.pPos.Xd;
                    it=it+1;


                    time = toc(tc)/(tcmax);
                    if toc(tc)>tcmax
                        time=1;
                    end
                    %Pegar informação da posição e velocidade real do robô
                    P.rGetSensorData(P);
                    %% Cálculo da curva:
                    N = 6;
                    Sum = 0;

                    for i=0:N-1
                        Sum = Sum + nchoosek(N-1,i)*(1-time).^(N-1-i).*(time.^i).*ControlPoints(:,i+1);
                    end
                    Curve = Sum;
                end
            end

                    %% Robot control
                    %Posição do ponto desejado:
                    P.pPos.Xd(1)= Curve(1);
                    P.pPos.Xd(2)= Curve(2);

                    % Pegando os dados do robo
            %         disp('.........................')
            %         disp(P.pPos.X(1))
                    P.rGetSensorData(P);
            %         disp(P.pPos.X(1))
                    P.pPos.Xtil = P.pPos.Xd - P.pPos.X;

        else            
            P.pPos.Xd(1:2) = [0.4589; 1.5];  
        end 
        
%         P.pPos.Xd(1:2) = [0.4589;0.9448]; 
        
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

J.pFlag = 0;
P.rDisableMotors;