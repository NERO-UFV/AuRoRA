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
    RI = RosInterface; 
    RI.rConnect('192.168.0.166');    
    
    P = RPioneer;    
    B = Bebop(1,'B');      
    
    NSBF = NullSpace3D;  % Null Space Object Instance            
    disp('Load All Class...............');
    
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


ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);


sampleTime = 1/30;
B.pPar.Ts = 1/30;
btnEmergencia = 0;

% Ajusted Gains - Teste 03 - 15/02/2019 - JV - Bom Resultado
% PF - ISE 13.52
NSBF.pPar.K1 = 1*diag([1 1 0 1 0.25 0.35]);                   % kinematic control gain - controls amplitude
NSBF.pPar.K2 = 0.1*diag([2.5 1 1 1 2.0 2.5]);                 % kinematic control gain - control saturation

% kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2
gains = [0.7 0.8 0.7 0.8 5 2; 1 12 1 12 1 4];   % teste  simulacao Referencia
pGains = [0.75 0.75 0.12 0.035];                % Ganhos do Compensador do Paioneer



%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_NSB')
Arq = fopen(['NSB_5Pos_' NomeArq '.txt'],'w');
cd(PastaAtual)



%% Robots initial pose
B.rGetSensorDataOpt;
P.rGetSensorDataOpt;


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
% Qd = [ 1   0   0   1.0   0   pi/3 ];
Qd = [ 1   0   0   1.0   0   pi/3 ];

% % Set Inicial Pioneer and Drone Position
%Xop = [-0.5 -0.5 0 0];
%P.rSetPose(Xop');
% 
% Xod = [0 2 1 0]';
% A.pPos.X(1:4) = Xod;

% Formation Weighted Error 
NSBF.mFormationWeightedError;

%% Formation initial error

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

% Send Comand do Take off 
% B.rTakeOff;    
% pause(4);

% Robots initial pose
B.rGetSensorDataOpt;
P.rGetSensorDataOpt;

% Formation initial pose
NSBF.pPos.Xr = [P.pPos.X(1:3); B.pPos.X(1:3)];

% Time variables initialization
timeout = 150;   % maximum simulation duration
index   = 1;     % Index Counter
time    = 120;    % time to change desired positions [s]
t  = tic;

ti = tic;    
tp = tic;
tc = tic;

t1 = tic;        % pioneer cycle
t2 = tic;        % ardrone cycle

disp('Inicialize Control Loop..............');
disp('');

%% Main Control Loop
try
    % Loop while error > erroMax
    while toc(t)< size(Qd,1)*(time) 
    
    if toc(tc) > sampleTime        
        tc = tic;      
        
        % Formation Error
        NSBF.pPos.Qtil = NSBF.pPos.Qd - NSBF.pPos.Q;
                
        % Robots initial pose
        B.rGetSensorDataOpt;
        P.rGetSensorDataOpt;
                        
        B.pSC.U = [B.pPos.X(4);B.pPos.X(5);B.pPos.X(9);B.pPos.X(12)]; % populates actual control signal to save data        
  
        % Formation Members Position        
        NSBF.mSetPose(P,B);                             
        
        % Get Sample time to integrate
        NSBF.SampleTime = toc(ti);
        ti = tic;                     
                
        
        %% Formation Control
        NSBF.mFormationControl;                 % Formação Convencional 
%       NSBF.mFormationPriorityControl;
%       NSBF.mPositionPriorityControl;


        % Desired position ...........................................
        NSBF.mInvTrans;
        
        % Pioneer
        P.pPos.Xd(1:3) = NSBF.pPos.Xr(1:3);         % desired position
        P.pPos.Xd(7:9) = NSBF.pPos.dXr(1:3);        % desired position
        
        % *Nota para Valentim: Para usar o fCompensadorDinamico tem que usar
        % a cinemática inversa abaixo para obter Ur. Eu acabei tirando isso
        % e usando o controlador dinâmico porque ele faz essa conversão.
        % Mas realmente o mais certo é usar o compensador. 
        P.sInvKinematicModel(P,NSBF.pPos.dXr(1:2));  % sem essa conversão o compensador não tem o valor de pSC.Ur, por isso o pioneer estava ficando parado
        
        % Drone
        B.pPos.Xda = B.pPos.Xd;               % save previous posture
        B.pPos.Xd(1:3) = NSBF.pPos.Xr(4:6);
        B.pPos.Xd(7:9) = NSBF.pPos.dXr(4:6);
        

        % Controlador 
        B.cInverseDynamicController;            % Bebop
        
        P = fCompensadorDinamico(P);
        
        % Get Performace Scores       
        NSBF.mFormationPerformace(sampleTime,toc(t),index);                                      
        
        % Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            B.pPos.Xd' B.pPos.X' B.pSC.Ud' B.pSC.U' NSBF.pPos.Qd' NSBF.pPos.Qtil' toc(t)]);
        fprintf(Arq,'\n\r');
        
        % Variable to feed plotResults function
        data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            B.pPos.Xd' B.pPos.X' B.pSC.Ud' B.pSC.U' NSBF.pPos.Qd' NSBF.pPos.Qtil' toc(t)];

        
        % Joystick command (priority)
        B = J.mControl(B);                      
        
        % Send control signals to robots      
        B.rCommand;
        P.rCommand;  
    
        % Increment Counter
        index = index + 1;   
                
    end
    
    
    % Draw robot
    if toc(tp) > inf
    %if toc(tp) > 0.5
        tp = tic;
        try
            delete(fig1);
            delete(fig2);
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
    
    % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
    if btnEmergencia ~= 0 || B.pFlag.EmergencyStop ~= 0 || B.pFlag.isTracked ~= 1
        disp('Bebop Landing through Emergency Command ');
        B.rCmdStop;
        B.rLand;
        break;
    end
        
    end
    
catch ME
    disp('Bebop Landing through Try/Catch Loop Command');
    disp('');
    disp(ME);
    B.rCmdStop;   
    P.rCmdStop;   
end


%% Close file and stop Program
fclose(Arq);

% Send 3 times Commands 1 second delay to Drone Land
for i=1:3
    disp("End Land Command");
    B.rCmdStop;    
    B.rLand    
end

%% Print Data and 
% figure;
close all;

NSB_PlotResults(data,1," - NSB - Formation Priority");
NSBF.mDisplayFormationPerformace(" - NSB - Formation Priority");
 
% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx




