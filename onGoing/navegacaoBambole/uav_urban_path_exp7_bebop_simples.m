%% Simulação do planejamento de trajetória para bambolês (wayposes)
% - Implementar seguimento de caminho
% - Analisar velocidade/curvatura no tempo, se não tiver ok corrigir
% - Distorcer curva para proteger bambolês passados
% A.pPar.Battery
% hardlim = z >= 1.5*0.63

% Boas práticas
close all
clear
clc

try
    fclose(instrfindall)
    % Rotina para buscar pasta raiz
    PastaAtual = pwd;
    PastaRaiz = 'AuRoRA 2018';
    cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
    addpath(genpath(pwd))
    
    disp('Diretório raiz e caminho bem definidos!')
catch
    disp('Diretório raiz errado!')
end

%% Definição da cena: ===================================================== 
close all
%Load Class
try
    % Load Classes
    RI = RosInterface; % 
    setenv('ROS_IP','192.168.0.158') % IP do computador que está rodando o código principal
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311') % IP do computador que está rodando o mestre
    RI.rConnect('192.168.0.146'); % Conectar ao mestre
    
    B = Bebop(1,'B1');
    
    % Joystick
    J = JoyControl;
    
    disp('  -------------------  Load Class Success  -------------------');
    
catch ME
    disp(' ');
    disp('  -------------------  Load Class Issues  -------------------');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    rosshutdown;
    return;
    
end

% Botão de Emergencia
figure(1)
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
B1 = ArDrone(1); % Bambolê Livre


% detect rigid body ID from optitrack
idB = getID(OPT,B,1);    % bebop ID on optitrack
idB1 = getID(OPT,ArDrone,1);     % ID do bambolê livre no optitrack

rb = OPT.RigidBody;          % read optitrack data
B = getOptData(rb(idB),B);   % get ardrone data
B1 = getOptData(rb(idB1),B1);   % get ardrone data

% Caminho inicial é uma circunferência:
% r = 0.75;       Crc.Vmax = .1;
% N = floor(((2*pi*r)/Crc.Vmax)/(1/30));
% time = linspace(0,2*pi,N);
% Crc.X = [r.*cos(time);r.*sin(time);1.15.*ones(1,N)];
% Crc.dX = [-r.*sin(time);r.*cos(time);zeros(1,N)];
% Crc.rho = 0; Crc.dXr = zeros(3,N);
% Crc.Pos = 1; Crc.Size = N; 
% Crc.Vmax = Curva.Vmax;


% Informações geratrizes: z1 = 0.8
p0 = [0,0.7,1.17]; n0 = [1,0,0];% ponto inicial 
Curva =[];      Curva.Vmax = .15;

figure(2)
axis([-4 3 -4 3 0 3])
view(228.1000,49.1520)
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
grid on

INDICE = 0;

[p0,n0,Bmb,Nb,Vb,Ub,Curva] = CorrigeCurva1(B,B1,Curva);
[CB,dCB,Curva,INDICE] = CurvaBmbExp1(p0,n0,Bmb,Nb,Vb,Ub,Curva);
%% Experimento ============================================================
% detect rigid body ID from optitrack
rb = OPT.RigidBody;          % read optitrack data
B = getOptData(rb(idB),B);   % get ardrone data

% Conectar Joystick
J = JoyControl;

% ArDrone Takeoff
disp('Start take off...');
B.rTakeOff;
pause(2)

% Bebop Controller Gains

gains = [  2     2     3     1.5     ...
           2     2     3     5     ...
           1     1     1     1.5     ...
           1     1     1     1        ];

       
XX = []; % Dados armazenados

t = tic;
tc = tic;
tp = tic;
rho = 0;

% Curva = Crc; CB = []; dCB=[];

% try
INDICE = Curva.Size -10;
while J.pFlag == 0
    if toc(tc) > 1/30
        tc = tic;
        tt = toc(t);

        clc
        fprintf('Posição no caminho: %i \n',Curva.Pos)
        fprintf('Percorrendo: %0.2g%% \n',(Curva.Pos/Curva.Size)*100)
        
        %             12        12      1   
        XX = [XX [B.pPos.Xd; B.pPos.X; tt]];
      
        % Get optitrack data        
        rb = OPT.RigidBody;  % read optitrack
        B = getOptData(rb(idB),B); % Atualiza dados ArDrone
        B1 = getOptData(rb(idB1),B1); % Atualiza dados bambolê livre
        


        Curva.rho = [Curva.rho,rho];
  
        % Controlador
        [B,Curva,rho] = cPathFollowing(B,Curva,Curva.Vmax);


        if Curva.Pos == Curva.Size-10
            Curva.Pos = INDICE;
        end

        
        B.pPos.Xr(1:3) = B.pPos.Xd(1:3);
        B.pPos.Xr(6) = B.pPos.Xd(6);
        B.pPos.Xr(7:9) = B.pPos.Xd(7:9);
        
        disp(B.pPos.X)
        
        
%         if rho>0.2
            B.pSC.Kinematics_control = 0;
%         else
%             A.pSC.Kinematics_control = 1;
%         end
        B.cInverseDynamicController_Compensador(gains);  % Controlador 
        
        if Curva.Pos==Curva.Size
            B.rLand;
        end
        B = J.mControl(B);
        
        B.rCommand;

        drawnow
        B.pFlag.EmergencyStop = 0;
        % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
        if btnEmergencia ~= 0 || J.pFlag == 1 || ...
                B.pFlag.EmergencyStop ~= 0 || B.pFlag.isTracked ~= 1 
            
            disp('Bebop Landing through Emergency Command ');
            B.rCmdStop;
            B.rLand;
            B.rCmdStop;
            B.rLand;
            B.rCmdStop;
            B.rLand;
            break;
        end
           
        flag=0;
        if J.pFlag == 1
            break 
        end
        disp(toc(tc))
    end
end
% catch ME
%     disp(ME)
%     B.rLand;
% end

%% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    disp("End Land Command");
    B.rCmdStop;
    B.rLand
    B.rCmdStop;
    B.rLand
    B.rCmdStop;
    B.rLand
    pause(1)
end

%% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

%% Resimulando ============================================================
% clc
disp('Calculado!')
disp('Desenhando cena...')
disp('Pronto para reproduzir.')
disp('Reproduzindo simulação...')
figure(2)
hold on
plot3(XX(13,:),XX(14,:),XX(15,:),'g-','LineWidth',2)
grid on
for tr = 1:5:size(XX,2)
    B.pPos.X = XX([13:24],tr);
%     B.mCADplot;
    drawnow
end

%% Salvar dados ===========================================================
dt = datestr(now,30);
dt = dt(end-6:end);
txt = ['uav_urban_path_exp7_bebop_simples',dt,'.mat'];
save(txt)
disp('SALVO!')

% Erro de caminho
figure
plot(Curva.rho,'b-','LineWidth',1.3)
hold on
plot(1:size(Curva.rho,2),(Curva.Vmax).*ones(1,size(Curva.rho,2)),'g--','LineWidth',1.3)
ylim([0,1.65*Curva.Vmax])
title('Erro de caminho')
xlabel('Pos')
ylabel('Til')

    
%     d1 = norm(A.pPos.X(1:3)-Bmb-dD.*(Nb/norm(Nb)));
%     d2 = norm(A.pPos.X(1:3)-Bmb+dD.*(Nb/norm(Nb)));
%     Dist = [d1,d2];
%     maior = max(Dist);
%     idx = find(Dist==maior);
