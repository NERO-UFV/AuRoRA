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
P.pPos.Xd(1:2) = [1; 1];
P.pPos.Xd(6) = 0;
%%
P.rEnableMotors;
% while J.pFlag ~= 0 && Condição
% Condição será o erro de posição do robô (Xd - X)
while J.pFlag == 0
if toc(Ta) > 0.1
    Ta = tic;
%     T_Atual = toc(T);

%   Posição desejada
    if norm(P.pPos.Xd(6) - P.pPos.X(6)) > 0.2
        P.pSC.Ud = [0; 0.2];
    elseif norm(P.pPos.Xd(1) - P.pPos.X(1)) > 0.2
%         P.pPos.Xd(1) = 1;
        P.pSC.Ud = [0.1; 0]; 
%         P.pPos.Xd = P.pPos.X;
%         P.pPos.Xd(1:2) = [0; 0.5];
%     elseif norm(P.pPos.Xd(6) - P.pPos.X(6)) > 0.1
% %         P.pPos.Xd(6) = 0;
%         P.pSC.Ud = [0; 0.1];
    elseif norm(P.pPos.Xd(2) - P.pPos.X(2)) > 0.2
        P.pPos.Xd(6) = pi/2;
%         P.pPos.Xd(2) = 1;
        P.pSC.Ud = [0.1; 0]; 
        
    else
        break
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






