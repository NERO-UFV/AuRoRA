%% Controle de Formação Baseado em Espaço Nulo
% ICUAS 2019
% Mauro Sérgio Mafra e Sara Jorge e Silva

%
% Referência da formação: Pionner
%
% Tarefa prioritária sendo controle de forma    (subíndice f) 
% Tarefa secundária sendo o controle de posição (subíndice p)
%
% Tarefa 1: qp = [rhof alphaf betaf]'   Jf(x): 3 últimas linhas
% Tarefa 2: qf = [xf yf zf]'            Jp(x): 3 primeiras linhas
% Projeção no espaço nulo da primeira tarefa: (I-(invJf*Jf))
% Sendo J(x) a Jacobiana da transformação direta e invJ sua pseudo inversa
% 
% Lei de Controle 1: qRefPontoi = qPontoi + Ki*qTili
% Lei de Controle 2: qRefPontoi = qPontoi + Li*tanh(pinv(Li)*Ki*qTili)
% Para formação desejada constante -> qPontoi = 0
%

% Resetar 
clear all;   close all;
warning off; clc;

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
    P = Pioneer3DX(1);  % Pioneer Instance
    A = ArDrone(2);     % Ardrone Instance        
    NSBF = NullSpace3D;  % Null Space Object Instance
    
    disp('Load All Class...............');
catch ME
    disp('Load Class Issues');
    disp('');
    disp(ME);
end


% Inicilize Drone Object Parameters
A.pPar.Ts = 1/30;
Xod = [1 1 1 0]';
A.pPos.X(1:4) = Xod;


% Inicialize Pioneer 3DX Object Parameters
% Robot/Simulator conection
P.rConnect;
Xop = [0 0 0 0];
P.rSetPose(Xop');

% Inicialize NSB Object Parameters
NSBF.pPos.X  = [0;0;0;-2;1;0];    % real pose [x1 y1 z1 x2 y2 z2]
NSBF.pPos.Xr = [0;0;0;-2;1;0];    % reference pose [x1 y1 z1 x2 y2 z2]
NSBF.pPos.Qd = [2;3;0;1;pi/2;0]'; % Desired Position

NSBF.pPar.K1 = 1*diag([1.0 1.0 1.0 0.8 0.08 0.08]);       % kinematic control gain  - controls amplitude
NSBF.pPar.K2 = 1*diag([0.1 0.1 0.025 0.1 0.01 0.01]);     % kinematic control gain - control saturation


% kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2
gains = [0.7 0.8 0.7 0.8 5 2; 1 12 1 12 1 4];   % teste Simulacao

% Joystick
J = JoyControl;

%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_NSB')
Arq = fopen(['NSB_' NomeArq '.txt'],'w');
cd(PastaAtual)


%% Variable initialization
data = [];
% Desired formation [xf yf zf rhof alfaf betaf]
% LF.pPos.Qd = [2 -1 0 2 deg2rad(0) deg2rad(90)]';
% Qd = [ 3   2   0   1.5   0   pi/3;
%        2  -2   0   2     pi/2   pi/3;
%       -2  -3   0   1.5   pi/4   pi/3;
%       -2   1   0   2     pi/4   pi/3;
%        0   0   0   1.5   0   pi/3];
% Artigo LARS2018
Qd = [ 2   2   0   1.5   0   pi/2;
       2  -2   0   2     0   pi/2;
      -2  -2   0   2     0   pi/2;
      -2   2   0   1.5   0   pi/2;
       0   0   0   1.5   0   pi/2];


cont = 1;     % counter to change desired position through simulation
time = 20;    % time to change desired positions [s]

%% Configure simulation window
fig = figure(1);

% plot robots initial positions
plot3(P.pPos.X(1),P.pPos.X(2),P.pPos.X(3),'r^','LineWidth',0.8); hold on;
plot3(A.pPos.X(1),A.pPos.X(2),A.pPos.X(3),'b^','LineWidth',0.8);

for kk = 1:size(Qd,1)
    NSBF.pPos.Qd = Qd(kk,:)';
    
    NSBF.mInvTrans;
    plot3(Qd(:,1),Qd(:,2),Qd(:,3),'r.','MarkerSize',20,'LineWidth',2),hold on;
    plot3(NSBF.pPos.Xd(4),NSBF.pPos.Xd(5),NSBF.pPos.Xd(6),'b.','MarkerSize',20,'LineWidth',2);
    % plot  formation line
    xl = [NSBF.pPos.Xd(1)   NSBF.pPos.Xd(4)];
    yl = [NSBF.pPos.Xd(2)   NSBF.pPos.Xd(5)];
    zl = [NSBF.pPos.Xd(3)   NSBF.pPos.Xd(6)];
    
    pl = line(xl,yl,zl);
    pl.Color = 'k';
    pl.LineStyle = '--';
    pl.LineWidth = 1;
    
    axis([-4 4 -4 4 0 4]);
    view(75,15)
    % Draw robot
    % P.mCADplot(1,'k');             % Pioneer
    % A.mCADplot                     % ArDrone
    grid on
    drawnow
end
% pause(3)

%% Formation initial error
% Formation initial pose
NSBF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];

% Formation initial pose
NSBF.mDirTrans;

% Formation Error
NSBF.mFormationError;

%% First desired position
NSBF.pPos.Qd = Qd(1,:)';

% Robots desired pose
NSBF.mInvTrans;

% Formation Error
NSBF.pPos.Qtil = NSBF.pPos.Qd - NSBF.pPos.Q;

%% Simulation
% Maximum error permitted
% erroMax = [.1 .1 0 .1 deg2rad(5) deg2rad(5)];

% Time variables initialization
timeout = 30;   % maximum simulation duration
% timeout = size(Qd,1)*time + 30;
t  = tic;
tc = tic;
tp = tic;
t1 = tic;        % pioneer cycle
t2 = tic;        % ardrone cycle


disp('Inicialize Simulation..............');
disp('');

% Loop while error > erroMax
while toc(t)< size(Qd,1)*(time)  %abs(NSB.pPos.Qtil(1))>erroMax(1) || abs(NSB.pPos.Qtil(2))>erroMax(2) || ...
%         abs(NSB.pPos.Qtil(4))>erroMax(4)|| abs(NSB.pPos.Qtil(5))>erroMax(5) ...
%         || abs(NSB.pPos.Qtil(6))>erroMax(6) % formation errors
    
    if toc(tc) > 1/30
        
        tc = tic;
        %% Desired positions  / time
        if cont <= size(Qd,1)
            NSBF.pPos.Qd = Qd(cont,:)';
        end
        if toc(t)> cont*time          
            cont = cont + 1;            
        end
       
        % Formation Error
        NSBF.pPos.Qtil = NSBF.pPos.Qd - NSBF.pPos.Q;

        %% Acquire sensors data
        P.rGetSensorData;    % Pioneer
        A.rGetSensorData;    % ArDrone
        
        
        %% Control
        % Formation Members Position
        % NSBF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
        NSBF.mSetPose(P,A);
                
        % Formation Control
%         NSBF.mFormationControl;  % Formação Convencional 
        NSBF.mFormationPriorityControl;
%         NSBF.mPositionPriorityControl;
               
        % Desired position ...........................................
        NSBF.mInvTrans;
        
        % Pioneer
        P.pPos.Xd(1:3) = NSBF.pPos.Xr(1:3);        % desired position
        P.pPos.Xd(7:9) = NSBF.pPos.dXr(1:3);        % desired position
        
        % *Nota para Valentim: Para usar o fCompensadorDinamico tem que usar
        % a cinemática inversa abaixo para obter Ur. Eu acabei tirando isso
        % e usando o controlador dinâmico porque ele faz essa conversão.
        % Mas realmente o mais certo é usar o compensador. 
        sInvKinematicModel(P,NSBF.pPos.dXr(1:2));  % sem essa conversão o compensador não tem o valor de pSC.Ur, por isso o pioneer estava ficando parado
        
        % Drone
        A.pPos.Xda = A.pPos.Xd;    % save previous posture
        A.pPos.Xd(1:3) = NSBF.pPos.Xd(4:6);
        A.pPos.Xd(7:9) = NSBF.pPos.dXr(4:6);
        %             A.pPos.Xd(6) = atan2(A.pPos.Xd(8),A.pPos.Xd(7)); % psi
        
        %             % Derivative (dPsi)
        %             if abs(A.pPos.Xd(6) - A.pPos.Xda(6)) > pi
        %                 if A.pPos.Xda(6) < 0
        %                     A.pPos.Xda(6) =  2*pi + A.pPos.Xda(6);
        %                 else
        %                     A.pPos.Xda(6) = -2*pi + A.pPos.Xda(6);
        %                 end
        %             end
        %             A.pPos.Xd(12) = (A.pPos.Xd(6) - A.pPos.Xda(6))/(1/30);  % dPsi
        % ............................................................
        % Dynamic compensantion - robot individual control
        
        A = cUnderActuatedController(A,gains);  % ArDrone
        A = J.mControl(A);           % joystick command (priority)
%         P = fDynamicController(P);     % Pioneer Dynamic Compensator
        P = fCompensadorDinamico(P);
        
        % Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' NSBF.pPos.Qd' NSBF.pPos.Qtil' toc(t)]);
        fprintf(Arq,'\n\r');
        
        % Variable to feed plotResults function
        data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' NSBF.pPos.Qd' NSBF.pPos.Qtil' toc(t)];
        
        % Send control signals to robots
        P.rSendControlSignals;
        A.rSendControlSignals;

    end
    
    
    % Draw robot
    if toc(tp) > 0.2
        tp = tic;
        try
            delete(fig1);
            delete(fig2);
        catch
        end
        P.mCADdel                      % Pioneer
        P.mCADplot(1,'k');             % Pioneer modelo bonitão
        A.mCADplot;                    % ArDrone
        
        % Percourse made
        fig1 = plot3(data(:,13),data(:,14),data(:,15),'r-','LineWidth',0.8); hold on;
        fig2 = plot3(data(:,41),data(:,42),data(:,43),'b-','LineWidth',0.8);
        
%         view(75,30)
        axis([-4 4 -4 4 0 4]);
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

%% Close file and stop robot
fclose(Arq);

% Send control signals
P.pSC.Ud = [0; 0];
P.rSendControlSignals;    % Pioneer

%% Plot results
% figure;
plotResults(data);
 
% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx




