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
    P = Pioneer3DX(1);  % Pioneer Instance
    A = ArDrone(1);     % Ardrone Instance        
    NSBF = NullSpace3D;  % Null Space Object Instance

    P2 = Pioneer3DX(2);  % Pioneer Instance
    A2 = ArDrone(2);     % Ardrone Instance        
    NSBF2= NullSpace3D;  % Null Space Object Instance

    P3 = Pioneer3DX(3);  % Pioneer Instance
    A3 = ArDrone(3);     % Ardrone Instance        
    NSBF3 = NullSpace3D;  % Null Space Object Instance
    
    disp('Load All Class...............');
    
catch ME
    disp(' ####   Load Class Issues   ####');
    disp('');
    disp(ME);
end

% Inicilize Drone Object Parameters
sampleTime = 1/30;
Xod = [1 1 1 0]';
A.pPos.X(1:4) = Xod;
A.pPar.Ts = 1/30;

A2(1,1).pPos.X(1:4) = Xod;
A2.pPar.Ts = 1/30;

A3.pPos.X(1:4) = Xod;
A3.pPar.Ts = 1/30;

% Inicialize Pioneer 3DX Object Parameters
% Robot/Simulator conection
%P.rConnect;

% Inicialize NSB Object Parameters
NSBF.pPos.X   = [4;-3;0;-4;-3;1];    % real pose [x1 y1 z1 x2 y2 z2]
NSBF2.pPos.X  = [4;-3;0;-4;-3;1];    % real pose [x1 y1 z1 x2 y2 z2]
NSBF3.pPos.X  = [4;-3;0;-4;-3;1];    % real pose [x1 y1 z1 x2 y2 z2]


% ganhos originais
% NSBF.pPar.K1 = 1*diag([0.8 0.8 0.5 0.02 0.015 0.03]);  % kinematic control gain  - controls amplitude
% NSBF.pPar.K2 = 1*diag([0.1 0.1 0.5 0.5 0.5 0.5]);      % kinematic control gain - control saturation

% NSBF.pPar.K1 = 2*diag([1.0 1.0 1.0 0.8 0.08 0.08]);       % kinematic control gain  - controls amplitude
% NSBF.pPar.K2 = 1*diag([0.1 0.1 0.025 0.1 0.01 0.01]);     % kinematic control gain - control saturation

% Simulation Gain
% NSBF.pPar.K1 = 1*diag([1 1 2 0.9 0.25 0.5]);            % kinematic control gain  - controls amplitude
% NSBF.pPar.K2 = 1*diag([0.1 0.1 0.1 0.1 0.1 0.1]);     % kinematic control gain - control saturation

% Test Gain
NSBF.pPar.K1 =   1*diag([1 1 0 1 0.35 0.25]);
NSBF.pPar.K2 = 0.1*diag([2.5 1 1 1 2.0 2.5]);

% Copy Gain to other simulations
NSBF2.pPar.K1 = NSBF.pPar.K1;      % kinematic control gain  - controls amplitude
NSBF2.pPar.K2 = NSBF.pPar.K2;      % kinematic control gain - control saturation

NSBF3.pPar.K1 = NSBF.pPar.K1;      % kinematic control gain  - controls amplitude
NSBF3.pPar.K2 = NSBF.pPar.K2;      % kinematic control gain - control saturation


% kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2
gains  = [0.7 0.8 0.7 0.8 5 2; 1 12 1 12 1 4];   % teste  simulacao Referencia
pGains = [0.75 0.75 0.12 0.035];                % Ganhos do Compensador do Paioneer

% Joystick
J = JoyControl;

%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_NSB')
Arq = fopen(['NSB_5Pos_' NomeArq '.txt'],'w');
cd(PastaAtual)


%% Variable initialization
data  = [];
data2 = [];
data3 = [];

% Desired formation [xf yf zf rhof alfaf betaf]
% LF.pPos.Qd = [2 -1 0 2 deg2rad(0) deg2rad(90)]';
% Qd = [ 3   2   0   1.5   0   pi/3;
%        2  -2   0   2     pi/2   pi/3;
%       -2  -3   0   1.5   pi/4   pi/3;
%       -2   1   0   2     pi/4   pi/3;
%        0   0   0   1.5   0   pi/3];

% Artigo LARS2018
% Qd = [ 0   3   0   1   -pi/2   pi/3 ];
Qd = [ 0   3.5   0   1.0   0   pi/2 ];

Xop = [-4 -4 0 0];
P.rSetPose(Xop');
P2.rSetPose(Xop');
P3.rSetPose(Xop');

Xod = [4 -4 1 0]';
A.pPos.X(1:4) = Xod;
A2.pPos.X(1:4) = Xod;
A3.pPos.X(1:4) = Xod;

% A.pPos.X(1) = 1;
% A.pPos.X(2) = -1;
% A.pPos.X(3) = 1;
% 
% A2.pPos.X(1) = 1;
% A2.pPos.X(2) = -1;
% A2.pPos.X(3) = 1;
% 
% A3.pPos.X(1) = 1;
% A3.pPos.X(2) = -1;
% A3.pPos.X(3) = 1;


index = 1;    % Index Counter
time = 60;    % time to change desired positions [s]

%% Configure simulation window
fig = figure(1);

% plot robots initial positions
plot3(P.pPos.X(1),P.pPos.X(2),P.pPos.X(3),'r^','LineWidth',0.8); hold on;
plot3(A.pPos.X(1),A.pPos.X(2),A.pPos.X(3),'k^','LineWidth',0.8);

% plot robots initial positions
plot3(P2.pPos.X(1),P2.pPos.X(2),P2.pPos.X(3),'r^','LineWidth',0.8); hold on;
plot3(A2.pPos.X(1),A2.pPos.X(2),A2.pPos.X(3),'k^','LineWidth',0.8);

% plot robots initial positions
plot3(P3.pPos.X(1),P3.pPos.X(2),P3.pPos.X(3),'r^','LineWidth',0.8); hold on;
plot3(A3.pPos.X(1),A3.pPos.X(2),A3.pPos.X(3),'k^','LineWidth',0.8);


for kk = 1:size(Qd,1)
    % 
    NSBF.pPos.Qd = Qd(kk,:)';    
    NSBF2.pPos.Qd = Qd(kk,:)';
    NSBF3.pPos.Qd = Qd(kk,:)';
    
    % Get Inverse Matrix 
    NSBF.mInvTrans;
    NSBF2.mInvTrans;
    NSBF3.mInvTrans;
    
    plot3(Qd(:,1),Qd(:,2),Qd(:,3),'r.','MarkerSize',20,'LineWidth',2),hold on;
  
    plot3(NSBF.pPos.Xd(4),NSBF.pPos.Xd(5),NSBF.pPos.Xd(6),'b.','MarkerSize',20,'LineWidth',4);        
    plot3(NSBF2.pPos.Xd(4),NSBF2.pPos.Xd(5),NSBF2.pPos.Xd(6),'b.','MarkerSize',20,'LineWidth',4);        
    plot3(NSBF3.pPos.Xd(4),NSBF3.pPos.Xd(5),NSBF3.pPos.Xd(6),'b.','MarkerSize',20,'LineWidth',4);

    
    % plot  formation line
    xl = [NSBF.pPos.Xd(1)   NSBF.pPos.Xd(4)];
    yl = [NSBF.pPos.Xd(2)   NSBF.pPos.Xd(5)];
    zl = [NSBF.pPos.Xd(3)   NSBF.pPos.Xd(6)];
    
    pl = line(xl,yl,zl);
    pl.Color = 'k';
    pl.LineStyle = '--';
    pl.LineWidth = 1;
    
    axis([-3 3 -3 3 0 3]);
    view(75,15)    
    
    % Draw robot
    hold on;    
        P.mCADplot(1,'k');             % Pioneer
        A.mCADplot                     % ArDrone    
        
        P2.mCADplot(1,'k');             % Pioneer
        A2.mCADplot                     % ArDrone    

        P3.mCADplot(1,'k');             % Pioneer
        A3.mCADplot                     % ArDrone    

    hold off;

    grid on
    drawnow
end
pause(3)

%% Formation initial error
% Formation initial pose
NSBF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
NSBF2.pPos.X = [P2.pPos.X(1:3); A2.pPos.X(1:3)];
NSBF3.pPos.X = [P3.pPos.X(1:3); A3.pPos.X(1:3)];

% Formation initial pose
NSBF.mDirTrans;
NSBF2.mDirTrans;
NSBF3.mDirTrans;

% Formation Error
NSBF.mFormationError;
NSBF2.mFormationError;
NSBF3.mFormationError;

% Formation Weighted Error 
NSBF.mFormationWeightedError;
NSBF2.mFormationWeightedError;
NSBF3.mFormationWeightedError;

%% First desired position
NSBF.pPos.Qd = Qd(1,:)';
NSBF2.pPos.Qd = Qd(1,:)';
NSBF3.pPos.Qd = Qd(1,:)';

% Robots desired pose
NSBF.mInvTrans;
NSBF2.mInvTrans;
NSBF3.mInvTrans;

% Formation Error
NSBF.pPos.Qtil = NSBF.pPos.Qd - NSBF.pPos.Q;
NSBF2.pPos.Qtil = NSBF2.pPos.Qd - NSBF2.pPos.Q;
NSBF3.pPos.Qtil = NSBF3.pPos.Qd - NSBF3.pPos.Q;

%% Simulation
% Maximum error permitted
% erroMax = [.1 .1 0 .1 deg2rad(5) deg2rad(5)];

% Time variables initialization
timeout = 150;   % maximum simulation duration
% timeout = size(Qd,1)*time + 30;
t  = tic;
tc = tic;
tp = tic;
t1 = tic;        % pioneer cycle
t2 = tic;        % ardrone cycle
ti = tic;

disp('Inicialize Simulation..............');
disp('');

% Loop while error > erroMax
while toc(t)< size(Qd,1)*(time)  %abs(NSB.pPos.Qtil(1))>erroMax(1) || abs(NSB.pPos.Qtil(2))>erroMax(2) || ...
%         abs(NSB.pPos.Qtil(4))>erroMax(4)|| abs(NSB.pPos.Qtil(5))>erroMax(5) ...
%         || abs(NSB.pPos.Qtil(6))>erroMax(6) % formation errors
    
    if toc(tc) > sampleTime        
        tc = tic;      
        
        % Formation Error
        NSBF.pPos.Qtil = NSBF.pPos.Qd - NSBF.pPos.Q;
        NSBF2.pPos.Qtil = NSBF2.pPos.Qd - NSBF2.pPos.Q;
        NSBF3.pPos.Qtil = NSBF3.pPos.Qd - NSBF3.pPos.Q;

        %% Acquire sensors data
        P.rGetSensorData;    % Pioneer
        A.rGetSensorData;    % ArDrone        
        
        P2.rGetSensorData;    % Pioneer
        A2.rGetSensorData;    % ArDrone        

        P3.rGetSensorData;    % Pioneer
        A3.rGetSensorData;    % ArDrone        
        
        
        %% Control
        % Formation Members Position        
        NSBF.mSetPose(P,A);
        NSBF2.mSetPose(P2,A2);
        NSBF3.mSetPose(P3,A3);
                        
        % Get Sample time to integrate
        NSBF.SampleTime = toc(ti);
        ti = tic;                                  % Reset Timer
                
        % Formation Control
        NSBF.mFormationControl;  % Formação Convencional                  
%        NSBF2.mFormationPriorityControl;
%        NSBF3.mPositionPriorityControl;

       NSBF2.mAuxFormationPriorityControl;
       NSBF3.mAuxPositionPriorityControl;



        % Desired position ...........................................
        NSBF.mInvTrans;
        NSBF2.mInvTrans;
        NSBF3.mInvTrans;
        
        % Pioneer
        P.pPos.Xd(1:3) = NSBF.pPos.Xr(1:3);         % desired position
        P.pPos.Xd(7:9) = NSBF.pPos.dXr(1:3);        % desired position
        
        P2.pPos.Xd(1:3) = NSBF2.pPos.Xr(1:3);         % desired position
        P2.pPos.Xd(7:9) = NSBF2.pPos.dXr(1:3);        % desired position

        P3.pPos.Xd(1:3) = NSBF3.pPos.Xr(1:3);         % desired position
        P3.pPos.Xd(7:9) = NSBF3.pPos.dXr(1:3);        % desired position
        
        
        % *Nota para Valentim: Para usar o fCompensadorDinamico tem que usar
        % a cinemática inversa abaixo para obter Ur. Eu acabei tirando isso
        % e usando o controlador dinâmico porque ele faz essa conversão.
        % Mas realmente o mais certo é usar o compensador. 
        sInvKinematicModel(P,NSBF.pPos.dXr(1:2));  % sem essa conversão o compensador não tem o valor de pSC.Ur, por isso o pioneer estava ficando parado
        
        sInvKinematicModel(P2,NSBF2.pPos.dXr(1:2));
        sInvKinematicModel(P3,NSBF3.pPos.dXr(1:2));
                
        % Drone
        A.pPos.Xda = A.pPos.Xd;    % save previous posture
        A.pPos.Xd(1:3) = NSBF.pPos.Xd(4:6);
        A.pPos.Xd(7:9) = NSBF.pPos.dXr(4:6);
        
        A = cUnderActuatedController(A,gains);  % ArDrone
        A = J.mControl(A);                      % joystick command (priority)                                     
        P = fCompensadorDinamico(P);

        
        
        % Drone 2
        A2.pPos.Xda = A2.pPos.Xd;    % save previous posture
        A2.pPos.Xd(1:3) = NSBF2.pPos.Xd(4:6);
        A2.pPos.Xd(7:9) = NSBF2.pPos.dXr(4:6);
        
        A2 = cUnderActuatedController(A2,gains);  % ArDrone
        A2 = J.mControl(A2);                      % joystick command (priority)                                     
        P2 = fCompensadorDinamico(P2);
       
        

        % Drone 3
        A3.pPos.Xda = A3.pPos.Xd;    % save previous posture
        A3.pPos.Xd(1:3) = NSBF3.pPos.Xd(4:6);
        A3.pPos.Xd(7:9) = NSBF3.pPos.dXr(4:6);
        
        A3 = cUnderActuatedController(A3,gains);  % ArDrone
        A3 = J.mControl(A3);                      % joystick command (priority)                                     
        P3 = fCompensadorDinamico(P3);
        
        
        
        % Get Performace Scores       
        NSBF.mFormationPerformace(sampleTime,toc(t),index);                                              
        NSBF2.mFormationPerformace(sampleTime,toc(t),index);                                      
        NSBF3.mFormationPerformace(sampleTime,toc(t),index);                                      
        
        
        % Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
                 A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' NSBF.pPos.Qd' NSBF.pPos.Qtil' toc(t)]);
        fprintf(Arq,'\n\r');
        
        % Variable to feed plotResults function
        data  = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
                 A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' NSBF.pPos.Qd' NSBF.pPos.Qtil' toc(t)];
        
        % Variable to feed plotResults function
        data2 = [data2; P2.pPos.Xd' P2.pPos.X' P2.pSC.Ud(1:2)' P2.pSC.U(1:2)' ...
                 A2.pPos.Xd' A2.pPos.X' A2.pSC.Ud' A2.pSC.U' NSBF2.pPos.Qd' NSBF2.pPos.Qtil' toc(t)];

        % Variable to feed plotResults function
        data3 = [data3; P3.pPos.Xd' P3.pPos.X' P3.pSC.Ud(1:2)' P3.pSC.U(1:2)' ...
                 A3.pPos.Xd' A3.pPos.X' A3.pSC.Ud' A3.pSC.U' NSBF3.pPos.Qd' NSBF3.pPos.Qtil' toc(t)];
             
             

        % Send control signals to robots
        P.rSendControlSignals;
        A.rSendControlSignals;

        P2.rSendControlSignals;
        A2.rSendControlSignals;

        P3.rSendControlSignals;
        A3.rSendControlSignals;              
        
        
        % Increment Counter
        index = index + 1;   
                
    end
    
    
    % Draw robot
    %if toc(tp) > inf
    if toc(tp) > 0.5
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
        P.mCADplot(1,'r');             % Pioneer modelo bonitão
        A.mCADplot;                    % ArDrone

        
        P2.mCADdel                      % Pioneer
        P2.mCADplot(1,'b');             % Pioneer modelo bonitão
        A2.mCADplot;                    % ArDrone

        P3.mCADdel                      % Pioneer
        P3.mCADplot(1,'g');             % Pioneer modelo bonitão
        A3.mCADplot;                    % ArDrone
        
        
        % Percourse made
       hold on;
            fig1 = plot3(data(:,13),data(:,14),data(:,15),'r-','LineWidth',2);
            fig2 = plot3(data(:,41),data(:,42),data(:,43),'r-','LineWidth',2);

            fig3 = plot3(data2(:,13),data2(:,14),data2(:,15),'k-','LineWidth',2);
            fig4 = plot3(data2(:,41),data2(:,42),data2(:,43),'k-','LineWidth',2);

            fig5 = plot3(data3(:,13),data3(:,14),data3(:,15),'g-','LineWidth',2);
            fig6 = plot3(data3(:,41),data3(:,42),data3(:,43),'g-','LineWidth',2);
        hold off;
        
%       view(75,30)
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

P2.pSC.Ud = [0; 0];
P2.rSendControlSignals;    % Pioneer

P3.pSC.Ud = [0; 0];
P3.rSendControlSignals;    % Pioneer


%% Plot results

%close all;

% figure;
disp(" ");
disp(" ");
disp("Controlador Convencional");
disp(" ");
NSB_PlotResults(data,1,"Conventional");
NSBF.mDisplayFormationPerformace("Conventional");

disp(" ");
disp("NSB - Formation Priority");
disp(" ");
NSB_PlotResults(data2,2,"NSB - Formation");
NSBF2.mDisplayFormationPerformace("NSB - Formation");

disp(" ");
disp("NSB - Position Priority");
disp(" ");
NSB_PlotResults(data3,3,"NSB - Position");
NSBF3.mDisplayFormationPerformace("NSB - Position");
 
 
% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx




