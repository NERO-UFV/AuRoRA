%% Tarefa 1

% Limpa as varaveis
clear all; 
close all;
warning off;
clc;

% Carrega o diretï¿½rio corrente e subdiretï¿½rios 

% %% Look for root folder
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))


%% Load Data Test
dataPath = strcat(pwd,'\ROS\Scripts\Dynamic model Identification\Data\');
wsName = 'WS_Calibracao.mat';
varCal = 'PSI';
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
    % Load Classes
A = ArDrone(1);
A.pPar.Ts = 1/30;
    
    OPT = OptiTrack;
    OPT.Initialize;
    % Initial Pose
idA = getID(OPT,ArDrone);
%     rb = OPT.RigidBody;            % read optitrack data
    
    %P = Pioneer3DX(1);  % Pioneer Instance
    
    % Joystick
    J = JoyControl;
       
    disp('################### Load Class Success #######################');
    

%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Drone %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Condiï¿½ï¿½es Iniciais de Posiï¿½ï¿½o do Drone
x(1) = 0;
y(1) = 0;
z(1) = 1;
phi(1) = 0;
phid = 0;

% Inicializa Parametros de configuraï¿½ï¿½o do teste
tmax = 300; % Tempo Simulacao em segundos
kA = 0.4;
kw = 0.1*2*pi;
% kw = abs(10*2*pi*sin(2*pi/300));

% Tempo de Amostragem
To = 1/30;
% =========================================================================
%t  = tic; % Tempo de simulacao
tc = tic; % Tempo de controle
tp = tic; % Tempo de plotagem e exibiï¿½ï¿½o
nLandMsg = 3;    % Numero de mensagens enviadas para pouso do drone
dLandMsg = 1; % Atraso de tempo entre messagens de pouso


%% Decolagem do Drone

% Assegura a decolagem vertical 
% Os valores de comando na decolagem sejam iguais a zero
A.pSC.Ud(1) = 0; % Esquerda/Direita [-1,1] (-) Move Drone para Esquerda
A.pSC.Ud(2) = 0; % Frente/Tras [-1,1] (-) Avanï¿½a - Move frente para baixo
A.pSC.Ud(3) = 0; % Velocidade Vertical [-1,1] (+) Eleva o drone
A.pSC.Ud(4) = 0; %                 

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
t  = tic; 


% Envia Comando de Decolagem para o Drone e aguarda estabilizaï¿½ï¿½o de
% altitude padrao (8s)
A.pPar.ip = '192.168.1.40';
A.rConnect;
A.rTakeOff;
pause(4);

disp(" ");
disp("Final da Estabilizaï¿½ao .... Controlador ON...")
disp(" ");
A.pPar.ti = tic;

% Eleva o drone para garantir distancia de oscilação
if elev
    while toc(t) < tz

        if toc(tc) > To            
            tc = tic;

            if toc(t) < (tz-2)
                VHref = 0.1;
                disp('elevacao');
            else
                 VHref = 0.0;
            end

%             B.rGetSensorData;
        rb = OPT.RigidBody;
            if rb(idA).isTracked
            A = getOptData(rb(idA),A);
            end
            
            A.pPar.ti = tic;


            A.pSC.Ud(1) = 0; % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
            A.pSC.Ud(2) = 0; % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda                        
            A.pSC.Ud(3) = 0; % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
            A.pSC.Ud(4) = 0; % Não Rotaciona 
            A.pSC.Ud(5) = 0; % Não Rotaciona
            A.pSC.Ud(6) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z 
            A.pSC.Ud = tanh(A.pSC.Ud);

            A.rSendControlSignals;  

         end


    end
end

%t  = tic; 
        rb = OPT.RigidBody;
            if rb(idA).isTracked
            A = getOptData(rb(idA),A);
            end
           
pPosAnt = A.pPos.X(6);
A.pPos.X(12) = (A.pPos.X(6) - pPosAnt)/To;
t  = tic;
v_odom = [];
%%
    while toc(t) < tmax
                      
       if toc(tc) > To            
            tc = tic;
                    
            countWhile = countWhile + 1; 
            
        rb = OPT.RigidBody;
            if rb(idA).isTracked
            A = getOptData(rb(idA),A);
            end
            v_odom = [v_odom; A.pPos.X(7) A.pPos.X(8) A.pPos.X(9) A.pPos.X(12)];
                    rb = OPT.RigidBody;
            if rb(idA).isTracked
            A = getOptData(rb(idA),A);
            end
           

            Vref = kA*sin(kw*toc(t));
            Vref = kA/(3 + 1 + .5)*(3*sin(kw*toc(t)) + 1*sin(3*kw*toc(t)) + .5*sin(5*kw*toc(t)));
                            
            A.pSC.Ud(1) = -Vref; % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
            A.pSC.Ud(2) = 0; % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda                        
            A.pSC.Ud(3) = 0; % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
            A.pSC.Ud(4) = 0; % Não Rotaciona 
            A.pSC.Ud(5) = 0; % Não Rotaciona
            A.pSC.Ud(6) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z 
            A.pSC.Ud = tanh(A.pSC.Ud);

            A.pPos.X(12) = (A.pPos.X(6) - pPosAnt)/To;
            pPosAnt = A.pPos.X(6);
            
            comandos = [comandos; A.pSC.Ud(1) A.pSC.Ud(2) A.pSC.Ud(3) A.pSC.Ud(6)];
            poses = [poses; A.pPos.X(1) A.pPos.X(2) A.pPos.X(3) A.pPos.X(6)];
            velocidades = [velocidades; A.pPos.X(7) A.pPos.X(8) A.pPos.X(9) A.pPos.X(12)];
            ArDrone = [ArDrone;A];
            tempo = [tempo toc(t)];            
            
            
            % Joystick Command Priority
            A = J.mControl(A);                    % joystick command (priority)
            
            % Envia Comando para o Drone
            A.rSendControlSignals;            
            drawnow;
        
       end
       
       % 
                   
    end
   

 

% Send 3 times Commands 1 second delay to Drone Land



%%  Save Data Test   

% Increase Data Test Serial Number
serialNumber = serialNumber + 1;

fileName = strcat('Calibracao_UFV_',toStringJSON(varCal),'_',...
         'Ka=',toStringJSON(kA),'_Kw=',toStringJSON(kw),'_' ,...
         'Serial_',toStringJSON(serialNumber),'.mat');
fullFileName = strcat(dataPath,fileName);
save(fullFileName,'comandos','poses','velocidades','tempo','ArDrone');

fileName = wsName;
fullFileName = strcat(dataPath,fileName);
save(fullFileName,'serialNumber');


%%


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

figure
hold on   
    plot(tempo,comandos(:,1))
    plot(tempo,comandos(:,2))
    plot(tempo,comandos(:,3))
    plot(tempo,comandos(:,4))
    
    title('Comandos')
    xlabel('Tempo(s)');
    ylabel('Vel');

    legend('Vel X','Vel Y','Vel Z', 'Vel Ang Z');        
hold off

figure
hold on
grid on
    plot(tempo,v_odom(:,2))
    plot(tempo,velocidades(:,2))

    title('Velocidades')
    xlabel('Tempo(s)');
    ylabel('Vel');

    legend('ODOM','OPT');        
hold off

