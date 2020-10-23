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
% ROS
r = ROSNetwork;
r.InitROS('/robot2','192.168.0.158')
r.SubscriberROS(r.node,'robot1/vel');

% Robot
P = Pioneer3DX;   % the ID must be different of the pioneer in the server machine to avoid conflict on NetDataShare

%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'land', ...
                         'Callback', 'btnEmergencia=1', ...
                         'Position', [50 50 400 300]);


%% Connect with robot or simulator MobileSim
P.rConnect;
tc = tic;
%% Experiment
while true % toc(t) < tsim
    
    if toc(tc) > 1/100
        tc = tic;
        
        % Read sensors data
        P.pSC.Ud = r.ReceiveROS('robot1/vel')';
%         P.pSC.Ud = [0 0]';   
        % Send commands to pioneer
        P.rSendControlSignals;


%% EMERGÊNCIA    
       drawnow
        
       if btnEmergencia ~= 0 
        P.pSC.Ud = [0 0]';     
        P.rSendControlSignals;

           break;
       end         
        
     end

end



