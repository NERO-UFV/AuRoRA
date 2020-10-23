clearvars
close all
clc

try
    fclose(instrfindall);
catch
end
%%
try
    % Load Classes
    RI = RosInterface;
    setenv('ROS_IP','192.168.0.158')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    
    % Inicializando o OptiTrack
    OPT = OptiTrack;    % Criando o OptiTrack
    OPT.Initialize;     % Iniciando o OptiTrack
    
    P = RPioneer(1,'P1'); % Pìoneer3DX Experimento
    P.pPar.a = 0;
    P.pPar.alpha = 0;
    P.rDisableMotors;
    
    % Definindo o ID do robô no OptiTrack
    idP = 1;
    
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
%%
% Temporizadores
Ta = tic;

%   OptiTrack
rb = OPT.RigidBody;

try
    if rb(idP).isTracked
        P = getOptData(rb(idP),P);
%         disp(P.pPos.X)
    end
catch
end

% P.pPos.X = [x; y; z; phi; theta; psi;
%             dx; dy; dz; dphi; dtheta; dpsi];

% Posição desejada
% P.pPos.Xd
P.pPos.Xd = P.pPos.X;
beta = atan((P.pPos.Xd(2)-P.pPos.X(2))/(P.pPos.Xd(1)-P.pPos.X(1)));
L=linspace(0 , 2*pi, 10);
i=1;
P.pPos.Xd(1:2) = [sin(L(i)); sin(2*L(i))];

%%
P.rEnableMotors;
% while J.pFlag ~= 0 && Condição
% Condição será o erro de posição do robô (Xd - X)
while J.pFlag == 0
    
if toc(Ta) > 0.1
    Ta = tic;
%     T_Atual = toc(T);

%   Posição desejada
    beta = atan2((P.pPos.Xd(2)-P.pPos.X(2)),(P.pPos.Xd(1)-P.pPos.X(1))) - P.pPos.X(6);
    while abs(beta) > pi
        if beta > 0
            beta = beta - 2*pi;
        else
            beta = beta + 2*pi;
        end
    end
    if beta > 0.2
        P.pSC.Ud = [0; 0.2];
    elseif beta < -0.2
        P.pSC.Ud = [0; -0.2];
    elseif norm(P.pPos.Xd(1:2) - P.pPos.X(1:2)) > 0.2 
        P.pSC.Ud = [0.3; 0];
    
    else 
        i = i+1;
        if i > 10
            break
        end
        P.pPos.Xd(1:2) = [sin(L(i)); sin(2*L(i))];
       
    end
    
%   Coletando sinais de posição e velocidade do robô
%   Odometria
%     P.rGetSensorData;

%   OptiTrack
    rb = OPT.RigidBody;             
    
    try
        if rb(idP).isTracked
            P = getOptData(rb(idP),P);
%             disp(P.pPos.X)
        end
    catch
    end
    
%   Sinal de controle 
%   P.pSC.Ud = [Linear; Angular];

%   Segurança #1
    P = J.mControl(P);
    
%   Enviar Comandos para o Pioneer
    P.rCommand;
    
%   Segurança #2
    P.pSC.Ud = [0; 0];
end
end

P.rDisableMotors;
%%






