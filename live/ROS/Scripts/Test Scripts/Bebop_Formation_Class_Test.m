%% Controle de Formação Baseado em Espaço Nulo
% ICUAS 2019
% Mauro Sérgio Mafra

%% Referência de Modelo Convencional (Marcos)
% Resetar 
clear all;   
close all;
warning off; 
clc;

%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

try
    fclose(instrfindall);
end

%% Load Class
try     
    % Load Classes
    RI = RosInterface;         
    RI.rConnect;    
    B = Bebop(1);               
    P = Pioneer3DX(1);  % Pioneer Instance    
    NSBF = NullSpace3D;  % Null Space Object Instance
    
    disp('### Load Class Success...');
    
catch ME
    disp(' #### Load Class Issues...');
    disp('');
    disp(ME);
    return;
end


% Inicialize Emergency Button
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'land', ...
                         'Callback', 'btnEmergencia=1', ...
                         'Position', [50 50 400 300]);


% Inicilize Drone Object Parameters
Xod = [1 1 1 0]';
B.pPos.X(1:4) = Xod;
B.pPar.Ts = 1/30;

% Inicialize NSB Object Parameters
NSBF.pPos.X  = [4;-2;0;-4;-2;1];    % real pose [x1 y1 z1 x2 y2 z2]

Xop = [0 0 0 0];
P.rSetPose(Xop');

Xod = [-1 0 1 0]';
B.pPos.X(1:4) = Xod;


% Ajusted Gains     
NSBF.pPar.K1 = 1*diag([1.6 1.6 0.4 0.5 0.35 0.35]);   % kinematic control gain  - controls amplitude
NSBF.pPar.K2 = 1*diag([0.1 0.1 0.1 0.2 0.2 0.2]);     % kinematic control gain - control saturation


% Joystick
J = JoyControl;



%% Variable initialization
data  = [];
Qd = [ 1   0   0   1   0   pi/3];


%% Configure simulation window
fig = figure(1);
    
% plot robots initial positions
%plot3(P.pPos.X(1),P.pPos.X(2),P.pPos.X(3),'r^','LineWidth',0.8); hold on;
plot3(B.pPos.X(1),B.pPos.X(2),B.pPos.X(3),'b^','LineWidth',0.8);

for kk = 1:size(Qd,1)
    % 
     NSBF.pPos.Qd = Qd(kk,:)';
     
%     % Get Inverse Matrix 
     NSBF.mInvTrans;

     plot3(Qd(:,1),Qd(:,2),Qd(:,3),'r.','MarkerSize',20,'LineWidth',2),hold on;
     plot3(NSBF.pPos.Xd(4),NSBF.pPos.Xd(5),NSBF.pPos.Xd(6),'b.','MarkerSize',20,'LineWidth',2);
%     
     % plot  formation line
     xl = [NSBF.pPos.Xd(1)   NSBF.pPos.Xd(4)];
     yl = [NSBF.pPos.Xd(2)   NSBF.pPos.Xd(5)];
     zl = [NSBF.pPos.Xd(3)   NSBF.pPos.Xd(6)];
    
    pl = line(xl,yl,zl);
    pl.Color = 'k';
    pl.LineStyle = '--';
    pl.LineWidth = 1;
    
    axis([-6 6 -6 6 0 4]);
    view(75,15)
    % Draw robot
    % P.mCADplot(1,'k');             % Pioneer
    % A.mCADplot                     % ArDrone
    grid on
    drawnow
end



%% Formation initial error
% Formation initial pose
NSBF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];

% Formation initial pose
NSBF.mDirTrans;

% Formation Error
NSBF.mFormationError;

% Formation Weighted Error 
NSBF.mFormationWeightedError;

%% First desired position
NSBF.pPos.Qd = Qd(1,:)';

% Robots desired pose
NSBF.mInvTrans;

% Formation Error
NSBF.pPos.Qtil = NSBF.pPos.Qd - NSBF.pPos.Q;


%% Simulation

nLandMsg = 5;    % Numero de mensagens enviadas para pouso do drone
dLandMsg = 1; % Atraso de tempo entre messagens de pouso

tmax = 60;
t  = tic;
tc = tic;
sampleTime = 1/5;
tp = tic;
drawTime = 0.5;

cont = 1;


disp('Inicialize Test ..............');
disp('');


try 
    
    % Loop while error > erroMax
    while toc(t)< tmax 

        if toc(tc) > sampleTime        
            tc = tic;                 
        
            % Desired positions  / time
            if cont <= size(Qd,1)
                NSBF.pPos.Qd = Qd(cont,:)';
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
      
    
        % Draw robot        
        if toc(tp) > drawTime
            tp = tic;
            try
                delete(fig1);
                delete(fig2);
                delete(fig3);
                delete(fig4);
                delete(fig5);
                delete(fig6);
            catch
            end
            P.mCADdel                      % Pioneer
            P.mCADplot(1,'k');             % Pioneer modelo bonitão
            B.mCADplot;                    % ArDrone

            % Percourse made
            fig1 = plot3(data(:,13),data(:,14),data(:,15),'r-','LineWidth',0.8); hold on;
            fig2 = plot3(data(:,41),data(:,42),data(:,43),'b-','LineWidth',0.8);

    %       view(75,30)
            axis([-6 6 -6 6 0 4]);
            grid on
            drawnow
            view(-21,30)
        end
        
        

        % Simulation timeout
        if toc(t)> size(Qd,1)*timeout
            disp('Timeout man!');
            break
        end
    end
    
    
catch ME
    % Send Land Command
    B.rLand;    
    
    % Fecha o cliente ROS
    B.RI.rDisconnect;   
    
    disp('');
    disp(ME);
    disp('');
    
end


% Finalizing Test

%% Close file and stop robot
fclose(Arq);

% Send control signals
P.pSC.Ud = [0; 0];
P.rSendControlSignals;    % Pioneer

% Send n times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    B.rLand
    pause(dLandMsg);
end

% Close ROS Interface
RI.rDisconnect;


%% Plot results

close all;

% figure;
NSB_PlotResults(data,1,"Conventional");
NSBF.mDisplayFormationPerformace("Conventional");

 
% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx



