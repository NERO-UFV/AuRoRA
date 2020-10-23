%% Tarefa 1

% Limpa as varaveis
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
dataPath = strcat(pwd,'\Dynamic Model Identification\Data\');
%wsName = 'WS_Calibracao.mat';
wsName = 'Calibracao_X3 - Long.mat';
fileName = wsName;
fullFileName = strcat(dataPath,fileName);

if exist(fullFileName)
    load(fullFileName);
else
    serialNumber = 0;
end
    


%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Drone  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
try
    fclose(instrfindall);
end


%% Load Class
try     
    % Load Classes
    RI = RosInterface;         
    %RI.rConnect('192.168.100.60');    
    RI.rConnect('172.20.24.236');    
    B = Bebop2(1);        
    
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


%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ButtonHandle = uicontrol('Style', 'PushButton', ...
%                          'String', 'land', ...
%                          'Callback', 'delete(gcbf)', ...
%                          'Position', [50 50 400 300]);
btnEmergencia = 0;
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
tmax = 120; % Tempo Simulacao em segundos
kA = 0.1;
kw = 0.5;

% Tempo de Amostragem
To = 1/30;

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
% altitude padrao (8s)
B.rTakeOff;
pause(4);

disp(" ");
disp("Final da Estabilizaï¿½ao .... Controlador ON...")
disp(" ");

countPrint = 0;
countWhile = 0;
countControl = 0;

% Inicializa o tempo de simulação

comandos = [];
poses = [];
velocidades = [];
tempo = [];
Bebop = [];
tz = 10;
elev = 0;

if elev
  try
    while toc(t) < tz

        if toc(tc) > To            
            tc = tic;

            if toc(t) < (tz-2)
                VHref = 0.05;
                disp('elevacao');
            else
                 VHref = 0.0;
            end

            B.rGetSensorData;

            B.pSC.Ud(1) = VHref; % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
            B.pSC.Ud(2) = 0; % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda                        
            B.pSC.Ud(3) = 0; % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
            B.pSC.Ud(4) = 0; % Não Rotaciona 
            B.pSC.Ud(5) = 0; % Não Rotaciona
            B.pSC.Ud(6) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z 

            B.rCommand;  

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
          return;
   end
end

%t  = tic; 
B.rGetSensorData
pPosAnt = B.pPos.X(6);
t  = tic; 
%%
 try    
    while toc(t) < tmax
                
        
       if toc(tc) > To            
            tc = tic;
                    
            countWhile = countWhile + 1; 

            B.rGetSensorData;

            Vref = kA*cos(kw*toc(t));
%             Vref = 0.3*sin(0.5*toc(t));
%             Vref = 0.3;
%             Vref = 0;
                            
            B.pSC.Ud(1) = Vref; % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
            B.pSC.Ud(2) = 0; % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda                        
            B.pSC.Ud(3) = 0; % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
            B.pSC.Ud(4) = 0; % Não Rotaciona 
            B.pSC.Ud(5) = 0; % Não Rotaciona
            B.pSC.Ud(6) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z 
            
            B.pPos.X(12) = (B.pPos.X(6) - pPosAnt)/To;
            pPosAnt = B.pPos.X(6);
            
            comandos = [comandos; B.pSC.Ud(1) B.pSC.Ud(2) B.pSC.Ud(3) B.pSC.Ud(6)];
            poses = [poses; B.pPos.X(1) B.pPos.X(2) B.pPos.X(3) B.pPos.X(6)];
            velocidades = [velocidades; B.pPos.X(7) B.pPos.X(8) B.pPos.X(9) B.pPos.X(12)];
            Bebop = [Bebop;B];

            tempo = [tempo toc(t)];
            
            
            if btnEmergencia ~= 0 
                disp('Bebop Landing');
                B.rLand;  
                break;
            end
            
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

% Increase Data Test Serial Number
serialNumber = serialNumber + 1;

fileName = strcat('Calibracao_PSI',toStringJSON(serialNumber),'.mat');
fullFileName = strcat(dataPath,fileName);
save(fullFileName,'comandos','poses','velocidades','tempo','Bebop');

fileName = wsName;
fullFileName = strcat(dataPath,fileName);
save(fullFileName,'serialNumber');


%%


% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Comando de Pouso ao final do programa");


%%
figure
hold on
    plot(tempo,poses(:,1))
    plot(tempo,poses(:,2))
    plot(tempo,poses(:,3))
    plot(tempo,poses(:,4))
    title('Posicoes')
    legend('Pos X','Pos Y','Pos Z', 'Ori Z');
    xlabel('Tempo(s)');
    ylabel('Pos');    
hold off


figure
hold on
    plot(tempo,velocidades(:,1))
    plot(tempo,velocidades(:,2))
    plot(tempo,velocidades(:,3))
    plot(tempo,velocidades(:,4))
    title('Velocidades')
    xlabel('Tempo(s)');
    ylabel('Vel');

    legend('Vel X','Vel Y','Vel Z', 'Vel Ang Z');        
hold off


