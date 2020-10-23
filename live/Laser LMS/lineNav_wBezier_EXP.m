%% Line reactive navigation EXP - Alexandre Caldeira (13/02/20)
%% Boas práticas
close all
clearvars
clc

%% Look for root directory
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
try
    cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
    addpath(genpath(pwd))
catch
    disp('Wrong root directory')
end

try
    fclose(instrfindall);
catch
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           Load ROS/Laser Class                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

try
    % Load Classes
    
    RI = RosInterface;
%     setenv('ROS_IP','192.168.0.158')
    setenv('ROS_IP','192.168.0.116')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    
%     % Inicializando o OptiTrack
%     OPT = OptiTrack;    % Criando o OptiTrack
%     OPT.Initialize;     % Iniciando o OptiTrack
    
    P = RPioneer(1,'P1'); % Pìoneer3DX Experimento
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

subLaser = rossubscriber('/scan','sensor_msgs/LaserScan');
Laser = [];
for ii = 1:100
    Laser = subLaser.LatestMessage;
end
Map = Laser.readCartesian;
figure(1)
% H1 = plot(-Map(:,2),Map(:,1),'.b');
hold on
grid on
axis equal
% axis([-20 20 -1 20])

%% Definindo o Robô
P.pPar.a = 0;
P.pPar.alpha = (0)*(pi/180);
pgains = [0.13 0.13 1];

H = HandlePushObj;

pause(1);

%% Inicializando variáveis para o controle
%Declarando o trace:
dados=[];
P.rGetSensorData;

%Parâmetros Line
LinMap=zeros(8,2); % 
% Map=[];
hmed = [];
Hist = [];
Curve =[];

%% Rotina da simulação:
t=tic;  ta=tic; tmax=120;
% tmax=80; %Percurso todo
k1=0.6; k2=0.4; it = 0;

P.rEnableMotors;
while toc(t)<tmax
    if toc(ta)>0.1
        ta=tic;
        it=it+1;
        %% Robot control
        %Disc following:
        
        %Line:
        Laser = subLaser.LatestMessage;    
        Map = Laser.readCartesian;
%         Map = P.pPos.X(1:2)' + Map;
        
        if isempty(Map)
            disp('Error: empty map!!!')
        else
         if (mod(it,3)==0)
            %Subsampling:
            for kk = 20:20:size(Map(:,2),1)
                X = Map(kk,1);%+P.pPos.X(1);
                Y = Map(kk,2);%+P.pPos.X(2);

                %Angular coeff:
                b = X\Y;
                Ylinha = X*b;

                LinMap(kk/20,1) = X(1);
                LinMap(kk/20,2) = Ylinha(1);
            end
            
            XX = LinMap(:,1);
            YY = LinMap(:,2);
            b1 = XX\YY;
            Yreta = XX*b1;

            med = mean(LinMap);
            
            hold on
            plot(med(1),med(2),'r*')
            hmed = [hmed;med];
            
            Curve = H.BezierCurve(hmed',toc(t)/tmax);
            
            %Posição do ponto médio:
            if toc(t)>10
                P.pPos.Xd(1)= med(1);
                P.pPos.Xd(2)= med(2);
            else
                P.pPos.Xd(1)= Curve(1);
                P.pPos.Xd(2)= Curve(2);
            end
            
            
            
            p1=plot(LinMap(:,1),LinMap(:,2),'b--');
%             p1 = Laser.plot;
            drawnow
         end
        end

%         if ~isempty(hmed)
%             Curve = H.BezierCurve(hmed',toc(t)/tmax);
% 
%             %Posição do ponto médio:
%             if toc(t)>10
%                 P.pPos.Xd(1)= med(1);
%                 P.pPos.Xd(2)= med(2);
%             else
%                 P.pPos.Xd(1)= Curve(1);
%                 P.pPos.Xd(2)= Curve(2);
%             end
%         end

        %Pegar informação da posição e velocidade real do robô
        P.rGetSensorData;
        
        P.pPos.Xd(1:2) = P.pPos.Xd(1:2) + P.pPos.X(1:2);
        
%         P.pPos.X(6)= atan2(P.pPos.X(2)-P.pPos.Xc(2),P.pPos.X(1)-P.pPos.Xc(1));
%         P.pPos.Xd(7)= 0;
%         P.pPos.Xd(8)= 0;
        
%         K=[cos(P.pPos.X(6)) -0.15*sin(P.pPos.X(6));...
%             sin(P.pPos.X(6)) 0.15*cos(P.pPos.X(6))];
       
        %Pegar informação da posição e velocidade real do robô
%         P.rGetSensorData;
               
        % Controle
%         P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
%               
%         P.pSC.Ud=K\(P.pPos.Xd([7 8])+k1*tanh(k2*P.pPos.Xtil([1 2])));
        P = fKinematicControllerExtended(P,pgains);

        % Armazenar dados da simulação
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];

        % Enviar sinais de controle para o robô
        P.rCommand;
        Hist = [P.pPos.Xc,Hist];
        disp(P.pPos.Xd(1:2))
    end
end
P.rDisableMotors;

% P.pSC.Ud([1 2]) = 0;
% P.rSendControlSignals(P,1);

%% Desconecta Matlab e V-REP
% P.vDisconnect;





