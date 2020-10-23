%% RECEBE COMANDOS PELA REDE E ENVIA DADOS DOS SENSORES
% Este programa apenas recebe comandos enviados de outro pc e os repassa ao
% pioneer. Além disso, envia para rede dados dos sensores do robô (u e w)

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
P = Pioneer3DX(2);   % the ID must be diferent of the pioneer in the server machine to avoid conflict on NetDataShare
J = JoyControl;
% Network data share
Rede =  NetDataShare;

%% Connect with robot or simulator MobileSim
P.rConnect;

%% Network communication check
% Only enter the loop after exchange data with server machine
tm = tic;
while true
    
    if isempty(Rede.pMSG.getFrom)
        Rede.mSendMsg(P);
        if toc(tm) > 0.100
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        end
        
    elseif length(Rede.pMSG.getFrom) > 1
        
        if isempty(Rede.pMSG.getFrom{1})
            Rede.mSendMsg(P);
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
X = Rede.pMSG.getFrom{1}(14+(1:12));  % get position from network (optitrack)
P.rSetPose(X([1 2 3 6]));             % set initial position

%% Variables initialization
% Time variables
tsim = 120;   % maximum simulation time
t    = tic;   % simulation current time
tc   = tic;   % sample time to send data to robot
tr   = tic;   % timer to reset pioneer pose
% Simulation data
data = [];    % save simulation data

%% Simulation
while true % toc(t) < tsim
    
    if toc(tc) > 1/30
        tc = tic;
        
        % Read sensors data
        P.rGetSensorData;
        
        % Send data to network
        Rede.mSendMsg(P);
        
        % Read Network
        Rede.mReceiveMsg;
        
        % Assign variables
        if ~isempty(Rede.pMSG.getFrom{1})
            
            P.pPos.Xd = Rede.pMSG.getFrom{1}(2+(1:12));    % desired [for plot purposes only]
            P.pSC.Ud  = Rede.pMSG.getFrom{1}(27:28);       % control signal
            X         = Rede.pMSG.getFrom{1}(14+(1:12));   % traveled (optitrack) [to correct odometry]
            
        end
        
        % Send commands to pioneer
%         P = J.mControl(P);
        P.rSendControlSignals;
        
        %         % Save data to plot
        %         data = [data; P.pSC.Ud' P.pSC.U' P.pPos.X(1:2)' X(1:2)' P.pPos.Xd'];
        %
     end
    % Reset pioneer position periodically
    %     if toc(tr)> 10 && ~isempty(X)
    if toc(tr)> 1/30 && ~isempty(X)
        
        X(3) = 0;                             % z value from optitrack may be different non-zero
        P.rSetPose(X([1 2 3 6]));             % assign initial position
    end
    
end

%%  Stop
% Zera velocidades do robô
P.pSC.Ud = [0 ; 0];
P.rSendControlSignals;

%% Results
% subplot(211)
% plot(data(:,1)),hold on;
% plot(data(:,3)), legend('Ud Linear ','Real')
% subplot(212)
% plot(data(:,2)),hold on
% plot(data(:,4)), legend('Ud Angular ','Real')
% % figure;
% % plot(data(:,5),data(:,6)), hold on
% % plot(data(:,7),data(:,8)),legend('Sensor','Optitrack')
%
% % End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

