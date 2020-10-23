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
%%
% Temporizadores
t1 = 8;
t2 = 5;
T_Execucao = 5*t1 + 4*t2 ;

T = tic;
    
    
    
    
    
%%
P.rEnableMotors;
while toc(T) < T_Execucao && J.pFlag == 0
    T_Atual = toc(T);
    
%   Sinal de controle 
%   P.pSC.Ud = [Linear; Angular];
    if toc(T) < t1
        P.pSC.Ud = [0; -0.2];
    elseif toc(T) < t1 + t2
        P.pSC.Ud = [0.2; 0];
    elseif toc(T) < 2*t1 + t2
        P.pSC.Ud = [0 ; 0.2];
    elseif toc(T) < 2*t1 + 2*t2
        P.pSC.Ud = [0.2; 0];
    elseif toc(T) < 4*t1 + 2*t2
        P.pSC.Ud = [0 ; 0.2];
    elseif toc(T) < 4*t1 + 3*t2
        P.pSC.Ud = [0.2; 0];
    elseif toc(T) < 5*t1 + 3*t2
        P.pSC.Ud = [0 ; 0.2];
    elseif toc(T) < 5*t1 + 4*t2
        P.pSC.Ud = [0.2 ; 0];
    end
%   Segurança #1
    P = J.mControl(P);
    
%   Enviar Comandos para o Pioneer
    P.rCommand;
    
%   Segurança #2
    P.pSC.Ud = [0; 0];
end
P.rDisableMotors;
%%






