%% RECEBE COMANDOS PELA REDE E ENVIA DADOS DOS SENSORES
% Este programa apenas recebe comandos enviados de outro pc e os repassa ao
% ArDrone. Além disso, envia para rede dados dos sensores do robô (u e w)


clear
close all
clc
% Fecha todas possíveis conexões abertas
try
    fclose(instrfindall);
catch
end

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Classes initialization
% Robot
A = ArDrone(2);   % the ID must be diferent of the robot in the server machine to avoid conflict on NetDataShare

% Joystick
J = JoyControl;

% Network data share
Rede = NetDataShare;

%% Connect with robot
A.rConnect;
A.rTakeOff;
% pause(4);

%% Network communication check
% Only enter the loop after exchange data with server machine
tm = tic;
while true
    
    if isempty(Rede.pMSG.getFrom)
        Rede.mSendMsg(A);
        if toc(tm) > 1/30
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        end
        
    elseif length(Rede.pMSG.getFrom) > 1
        
        if isempty(Rede.pMSG.getFrom{1})
            Rede.mSendMsg(A);
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        else
            break
        end
    end
end
clc
disp('Data received. Continuing the program...');

%% Define robot initial position
A.pPos.X = Rede.pMSG.getFrom{1}(14+(1:12));  % get position from network (optitrack)

%% Variables initialization
% Time variables
tsim = 18;   % maximum simulation time
t    = tic;   % simulation current time
tc   = tic;   % sample time to send data to robot

%% Simulation
while toc(t) < tsim 
    if toc(tc) > 1/30
        tc = tic;
        
        % Read sensors data
%         A.rGetSensorData;
        
        % Send data to network
        Rede.mSendMsg(A);
        
        % Read Network
        Rede.mReceiveMsg;
        
        % Assign variables
        if ~isempty(Rede.pMSG.getFrom{1})
            
            A.pPos.Xd = Rede.pMSG.getFrom{1}(2+(1:12));    % desired [for plot purposes only]
            A.pPos.X  = Rede.pMSG.getFrom{1}(14+(1:12));   % traveled (optitrack) [to correct odometry]
            A.pSC.Ud  = Rede.pMSG.getFrom{1}(27:30);       % desired control signal
            A.pSC.U   = Rede.pMSG.getFrom{1}(31:34);       % control signal
        else
            disp('No data received through network');
        end
        A = J.mControl(A);                      % joystick command (priority)
 
        % Send commands to pioneer
        A.rSendControlSignals;
                
    end   
end

%%  Stop
% Land drone
if A.pFlag.Connected == 1
    A.rLand;
end


% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx