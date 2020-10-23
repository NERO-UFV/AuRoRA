%% Tarefa 1

% Limpa as variï¿½vies
clear all; 
close all;
warning off;
clc;

% Carrega o diretï¿½rio corrente e subdiretï¿½rios 

%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


%% Load Data Test
dataPath = strcat(pwd,'\Test Scripts and Calibrations\Data\');
wsName = 'WS_Calibracao.mat';
fileName = wsName;
fullFilePath = strcat(dataPath,fileName);

if exist(fullFilePath)
    load(fullPath);
else
    serialNumber = 1;
end
    


%%

btnEmergencia = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Drone  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
try
    fclose(instrfindall);
end


%% Load Class
try     
    % Load Classes
    RI = RosInterface;         
    RI.rConnect;    
    B = Bebop2(1);        
    
    disp('### Load Class Success');
    
catch ME
    disp(' #### Load Class Issues ');
    disp('');
    disp(ME);
    rosshutdown;
    return;
end
%%


%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ButtonHandle = uicontrol('Style', 'PushButton', ...
%                          'String', 'land', ...
%                          'Callback', 'delete(gcbf)', ...
%                          'Position', [50 50 400 300]);

ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'land', ...
                         'Callback', 'btnEmergencia=1', ...
                         'Position', [50 50 400 300]);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Drone %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Condiï¿½ï¿½es Iniciais de Posiï¿½ï¿½o do Drone
x(1) = 0;
y(1) = 0;
z(1) = 1;
phi(1) = 0;
phid = 0;

% Inicializa Parametros de configuraï¿½ï¿½o do teste
tmax = 60; % Tempo Simulaï¿½ï¿½o em segundos
kA = 0.1;
kw = 0.5;

% Tempo de Amostragem
To = 1/5;

% =========================================================================
%t  = tic; % Tempo de simulacao
tc = tic; % Tempo de controle
tp = tic; % Tempo de plotagem e exibiï¿½ï¿½o
nLandMsg = 5;    % Numero de mensagens enviadas para pouso do drone
dLandMsg = 1; % Atraso de tempo entre messagens de pouso



%% Decolagem do Drone

% Assegura a decolagem vertical 
% Os valores de comando na decolagem sejam iguais a zero
B.pSC.Ud(1) = 0; % Esquerda/Direita [-1,1] (-) Move Drone para Esquerda
B.pSC.Ud(2) = 0; % Frente/Tras [-1,1] (-) Avanï¿½a - Move frente para baixo
B.pSC.Ud(3) = 0; % Velocidade Vertical [-1,1] (+) Eleva o drone
B.pSC.Ud(4) = 0; %                 

% Envia Comando de Decolagem para o Drone e aguarda estabilizaï¿½ï¿½o de
% altitude padrï¿½o (8s)
B.rTakeOff;
pause(4);

disp(" ");
disp("Final da Estabilizaï¿½ao .... Controlador ON...")
disp(" ");

countPrint = 0;
countWhile = 0;
countControl = 0;

% Inicializa o tempo de simulação
t  = tic; 
comandos = [];
poses = [];
velocidades = [];
tempo = [];


%%
 try    
    while toc(t) < tmax
        
       if toc(tc) > To            
            tc = tic;
                    
            countWhile = countWhile + 1;             
            B.rGetSensorData;                   

            Vref = 0.2*cos(0.5*toc(t));            
                            
            B.pSC.Ud(1) = 0;%Vref; % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
            B.pSC.Ud(2) = 0; % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda                        
            B.pSC.Ud(3) = 0; % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
            B.pSC.Ud(4) = 0; % Angulo do drone [-1,1] (+) rotaciona para Direita em torno do Eixo Z 
            B.pSC.Ud(5) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda 
            B.pSC.Ud(6) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z 

            
            comandos = [comandos; B.pSC.Ud(1) B.pSC.Ud(2) B.pSC.Ud(3) B.pSC.Ud(6)];
            poses = [poses; B.pPos.X(1) B.pPos.X(2) B.pPos.X(3) B.pPos.X(6)];
            velocidades = [velocidades; B.pPos.X(7) B.pPos.X(8) B.pPos.X(9) B.pPos.X(12)];

            tempo = [tempo toc(t)];
            
            
            % Envia Comando para o Drone
            B.rCommand;            
            drawnow;
        
       end
       
       if btnEmergencia ~= 0 
                disp('Bebop Landing');
                B.rLand;  
                break;
       end
               
    end
   
 catch Exception    
      B.rLand;          
      disp("Pouso forcado via comando da estrutura Try Catch");
      disp(toStringJSON(Exception));
 end


 

% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    B.rLand
    pause(dLandMsg);
end


%%  Save Data Test   

fileName = strcat('Calibracao_X',toStringJSON(serialNumber),'.mat');
fullFilePath = strcat(dataPath,fileName);
save(fullFilePath,'comandos','poses','velocidades','tempo');

fileName = wsName;
fullFilePath = strcat(dataPath,fileName);
save(fullFilePath,'serialNumber');

% Increase Data Test Serial Number
serialNumber = serialNumber + 1;

%%


% Close ROS Interface
RI.rDisconnect;
rosshutdown;


disp("Comando de Pouso ao final do programa");


%%
figure
plot(tempo,poses(:,1))
hold on
plot(tempo,poses(:,2))
plot(tempo,poses(:,3))
plot(tempo,poses(:,4))

figure
plot(tempo,velocidades(:,1))
hold on
plot(tempo,velocidades(:,2))
plot(tempo,velocidades(:,3))
plot(tempo,velocidades(:,4))
