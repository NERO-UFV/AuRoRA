% Positioning task

clear
close all
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

%% Definindo o Robô
P = RPioneer(1,'P1');
P.pPar.a = 0;
P.pPar.alpha = -0.5*pi/4;
%% Definindo a Figura que irá rodar a simulação
figure(1)
hold on
grid on
title('Simulação');
axis([-3 3 -3 3])
P.mCADplot2D('r')   % Visualização 2D
drawnow

% Tempo de esperar para início do experimento/simulação
dados = [];
pause(1)
P.rEnableMotors;
%% Tempo Real
tmax = 30;
tc = tic;
tp = tic;
t = tic;

%% Inicio da simulação
while (toc(t) <= tmax)
    % Sub-rotina de controle
    if toc(tc) > 0.1
        % Inicio da realimentação
        tc = tic;
        
        P.pPos.Xda     = P.pPos.Xd;
        if toc(t) > 15
            P.pPos.Xd(1:2) = [0.4105; 1.4724];
        else
            P.pPos.Xd(1:2) = [1.5160; -2.1254];
        end
        % Pegando os dados do robo
%         P.rGetSensorData;
        rb = OPT.RigidBody;             % read optitrack
        
        try
            if rb(idP).isTracked
                P = getOptData(rb(idP),P);
%                 disp(P.pPos.X)
            end
        catch
        end
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
       P = fKinematicControllerExtended(P);

       P = J.mControl(P);
        
       P.rCommand;

        % Armazenar dados da simulação
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];
          
        % Sub-rotina para plotar
        if toc(tp) > 0.1
            tp = tic;
            % Plot da simulação
            P.mCADdel
            % P.mCADplot(1,'r') % Visualização 3D
            P.mCADplot2D('r')   % Visualização 2D
            drawnow
        end
        
    end
end

% P.pSC.Ud([1 2]) = 0;
P.rCommand;

J.pFlag = 0;
P.rDisableMotors;


