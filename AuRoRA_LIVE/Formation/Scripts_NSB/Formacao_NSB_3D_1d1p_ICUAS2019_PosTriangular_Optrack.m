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
pause
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
    A = ArDrone(30);     % Ardrone Instance        
    NSBF = NullSpace3D;  % Null Space Object Instance
    
    disp('Load All Class...............');
    
catch ME
    disp(' ####   Load Class Issues   ####');
    disp('');
    disp(ME);
end

% Inicilize Drone Object Parameters
sampleTime = 1/30;
A.pPar.Ts = 1/30;


% Inicialize Pioneer 3DX Object Parameters
% Robot/Simulator conection
% P.rConnect;

% ganhos originais
% NSBF.pPar.K1 = 1*diag([0.8 0.8 0.5 0.02 0.015 0.03]);  % kinematic control gain  - controls amplitude
% NSBF.pPar.K2 = 1*diag([0.1 0.1 0.5 0.5 0.5 0.5]);      % kinematic control gain - control saturation

% Ajusted Gains Controlador Convencional - 14/02/2019
% NSBF.pPar.K1 =    1*diag([1 1 2 1.2 0.1 0.1]);          % kinematic control gain - controls amplitude
% NSBF.pPar.K2 = 0.12*diag([1 1 1 1 1 1]);                  % kinematic control gain - control saturation

% Ajusted Gains - Drone Não sobe
% NSBF.pPar.K1 =    1*diag([1 1 2 1.2 0.1 0.1]);          % kinematic control gain - controls amplitude
% NSBF.pPar.K2 = 0.12*diag([1 1 1 1 1 1]);                  % kinematic control gain - control saturation

% NSBF.pPar.K1 = 2*diag([1.0 1.0 1.0 0.8 0.08 0.08]);    % kinematic control gain  - controls amplitude
% NSBF.pPar.K2 = 1*diag([0.1 0.1 0.025 0.1 0.01 0.01]);  % kinematic control gain - control saturation

% NSBF.pPar.K1 = 1*diag([1 1 2 1 0.25 0.5]);             % kinematic control gain  - controls amplitude
% NSBF.pPar.K2 = 1*diag([0.1 0.1 0.1 0.1 0.1 0.1]);      % kinematic control gain - control saturation



% Ajusted Gains - Teste 01 - 14/02/2019
% NSBF.pPar.K1 = 1*diag([1 1 2 0.9 0.25 0.5]);             % kinematic control gain - controls amplitude
% NSBF.pPar.K2 = 1*diag([0.1 0.1 0.1 0.1 0.1 0.1]);        % kinematic control gain - control saturation

% Ajusted Gains - Teste 03 - 15/02/2019 - JV - Funcionou
% PF - ISE 22.5
% NSBF.pPar.K1 = 1*diag([1 1 0 1 0.025 0.25]);            % kinematic control gain - controls amplitude
% NSBF.pPar.K2 = 0.1*diag([1 1 1 1 2 2]);                 % kinematic control gain - control saturation

% Ajusted Gains - Teste 03 - 15/02/2019 - JV - Funcionou
% PF - ISE 22.5
% NSBF.pPar.K1 = 1*diag([1 1 0 1 0.025 0.25]);            % kinematic control gain - controls amplitude
% NSBF.pPar.K2 = 0.1*diag([1 1 1 1 2.5 2.5]);                 % kinematic control gain - control saturation


% Ajusted Gains - Teste 03 - 15/02/2019 - JV - Funcionou
% PF - ISE 21.7370
% NSBF.pPar.K1 = 1*diag([1 1 0 1 0.25 0.25]);                % kinematic control gain - controls amplitude
% NSBF.pPar.K2 = 0.1*diag([ 1 1 1 2.5 2.5]);                 % kinematic control gain - control saturation

% Ajusted Gains - Teste 03 - 15/02/2019 - JV - Funcionou
% PF - ISE 19.89
% NSBF.pPar.K1 = 1*diag([1 1 0 1 0.25 0.35]);                % kinematic control gain - controls amplitude
% NSBF.pPar.K2 = 0.1*diag([ 1 1 1 2.5 2.5]);                 % kinematic control gain - control saturation

% Ajusted Gains - Teste 03 - 15/02/2019 - JV - Bom Resultado
% PF - ISE 12.38
% NSBF.pPar.K1 = 1*diag([1 1 0 1 0.25 0.35]);            % kinematic control gain - controls amplitude
% NSBF.pPar.K2 = 0.1*diag([2.5 1 1 1 2.2 2.5]);                 % kinematic control gain - control saturation


% Ajusted Gains - Teste 03 - 15/02/2019 - JV - Bom Resultado
% PF - ISE 11.87
% NSBF.pPar.K1 = 1*diag([1 1 0 1 0.25 0.35]);            % kinematic control gain - controls amplitude
% NSBF.pPar.K2 = 0.1*diag([2.5 1 1 1 2.0 2.5]);                 % kinematic control gain - control saturation


% Ajusted Gains - Teste 03 - 15/02/2019 - JV - Bom Resultado
% PF - ISE 13.52
NSBF.pPar.K1 = 1*diag([1 1 0 1 0.25 0.35]);            % kinematic control gain - controls amplitude
NSBF.pPar.K2 = 0.1*diag([2.5 1 1 1 2.0 2.5]);                 % kinematic control gain - control saturation





% kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2
gains = [0.7 0.8 0.7 0.8 5 2; 1 12 1 12 1 4];   % teste  simulacao Referencia
pGains = [0.75 0.75 0.12 0.035];                % Ganhos do Compensador do Paioneer

% Joystick
J = JoyControl;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Network
Rede = NetDataShare;


%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_NSB')
Arq = fopen(['NSB_5Pos_' NomeArq '.txt'],'w');
cd(PastaAtual)



%% Network communication check
tm = tic;
while true
    
    if isempty(Rede.pMSG.getFrom)
        Rede.mSendMsg(P);
        if toc(tm) > 0.1
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        end
    elseif length(Rede.pMSG.getFrom) > 1
        if isempty(Rede.pMSG.getFrom{2})
            Rede.mSendMsg(P);
            
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
            
        else
            break
        end
    end
end
clc
disp('Data received. Continuing program...');


%% Robot/Simulator conection
A.rConnect;

%% Robots initial pose
% detect rigid body ID from optitrack
idP = getID(OPT,P);            % pioneer ID on optitrack
idA = getID(OPT,A);            % drone ID on optitrack

rb = OPT.RigidBody;            % read optitrack data
A = getOptData(rb(idA),A);     % get ardrone data
P = getOptData(rb(idP),P);     % get pioneer data


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


%% Configure simulation window
% fig = figure(1);
% 
% % plot robots initial positions
% plot3(P.pPos.X(1),P.pPos.X(2),P.pPos.X(3),'r^','LineWidth',0.8); hold on;
% plot3(A.pPos.X(1),A.pPos.X(2),A.pPos.X(3),'b^','LineWidth',0.8);

% Formation Weighted Error 
NSBF.mFormationWeightedError;

% 
% for kk = 1:size(Qd,1)
%     % 
%     NSBF.pPos.Qd = Qd(kk,:)';
%     
%     % Get Inverse Matrix 
%     NSBF.mInvTrans;
%     
%     plot3(Qd(:,1),Qd(:,2),Qd(:,3),'r.','MarkerSize',20,'LineWidth',2),hold on;
%     plot3(NSBF.pPos.Xd(4),NSBF.pPos.Xd(5),NSBF.pPos.Xd(6),'b.','MarkerSize',20,'LineWidth',2);
%     
%     % plot  formation line
%     xl = [NSBF.pPos.Xd(1)   NSBF.pPos.Xd(4)];
%     yl = [NSBF.pPos.Xd(2)   NSBF.pPos.Xd(5)];
%     zl = [NSBF.pPos.Xd(3)   NSBF.pPos.Xd(6)];
%     
%     pl = line(xl,yl,zl);
%     pl.Color = 'k';
%     pl.LineStyle = '--';
%     pl.LineWidth = 1;
%     
%     axis([-4 4 -4 4 0 3]);
%     view(75,15)
%     % Draw robot
%     % P.mCADplot(1,'k');             % Pioneer
%     % A.mCADplot                     % ArDrone
%     grid on
%     drawnow
% end

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


%% Simulation
% Maximum error permitted
% erroMax = [.1 .1 0 .1 deg2rad(5) deg2rad(5)];

% Time variables initialization
timeout = 150;   % maximum simulation duration
index   = 1;     % Index Counter
time    = 45;    % time to change desired positions [s]

% timeout = size(Qd,1)*time + 30;

% tp = tic;
% tc = tic;

% Send Comand do Take off 
pause(2)
for ii = 1:5
    A.rTakeOff;    
end
pause(4);

% disp("Start Take Off Timming....");
% while toc(tp) < 5        
%      if toc(tc) > sampleTime                      
%         % Joystick
%         A = J.mControl(A);   
%         A.rSendControlSignals;  
%         % Display reference
%         display([NSBF.pPos.Xr(4:6)';NSBF.pPos.Xd(4:6)']);
% 
%      end
% end
% disp("Taking Off End Time....");


% Get optitrack data
rb = OPT.RigidBody;             % read optitrack

% Ardrone
A = getOptData(rb(idA),A);
P = getOptData(rb(idP),P);

% Formation initial pose
NSBF.pPos.Xr = [P.pPos.X(1:3); A.pPos.X(1:3)];

tp = tic;
tc = tic;
t  = tic;
t1 = tic;        % pioneer cycle
t2 = tic;        % ardrone cycle

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

        %% Acquire sensors data
%         P.rGetSensorData;    % Pioneer
        %A.rGetSensorData;    % ArDrone  
                
        % Get network data
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{2}(29:30);  % current velocities (robot sensors)
            PX       = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
        end
        
        % Get optitrack data
        rb = OPT.RigidBody;             % read optitrack
        
        % Ardrone
        A = getOptData(rb(idA),A);
        A.pSC.U = [A.pPos.X(4);A.pPos.X(5);A.pPos.X(9);A.pPos.X(12)]; % populates actual control signal to save data
        P = getOptData(rb(idP),P);                                
  
        % Fo    rmation Members Position        
        NSBF.mSetPose(P,A);                             
        
        % Get Sample time to integrate
        NSBF.SampleTime = toc(ti);
        ti = tic;                     
                
        
        %% Formation Control
%       NSBF.mFormationControl;                 % Formação Convencional 
%       NSBF.mFormationPriorityControl;
%       NSBF.mPositionPriorityControl;


        NSBF.mFormationControl;
%       NSBF.mAuxPriorityControl;
%       NSBF.mAuxFormationPriorityControl;
%        NSBF.mAuxPositionPriorityControl;

        % Display reference
        
%         display([NSBF.pPos.Xr(4:6)';NSBF.pPos.Xd(4:6)']);
               
        % Desired position ...........................................
        NSBF.mInvTrans;
        
        % Pioneer
        P.pPos.Xd(1:3) = NSBF.pPos.Xr(1:3);         % desired position
        P.pPos.Xd(7:9) = NSBF.pPos.dXr(1:3);        % desired position
        
        % *Nota para Valentim: Para usar o fCompensadorDinamico tem que usar
        % a cinemática inversa abaixo para obter Ur. Eu acabei tirando isso
        % e usando o controlador dinâmico porque ele faz essa conversão.
        % Mas realmente o mais certo é usar o compensador. 
        sInvKinematicModel(P,NSBF.pPos.dXr(1:2));  % sem essa conversão o compensador não tem o valor de pSC.Ur, por isso o pioneer estava ficando parado
        
        % Drone
        A.pPos.Xda = A.pPos.Xd;    % save previous posture
        A.pPos.Xd(1:3) = NSBF.pPos.Xr(4:6);
        A.pPos.Xd(7:9) = NSBF.pPos.dXr(4:6);
        
%         A = cUnderActuatedController(A);
        A = cUnderActuatedController(A,gains);  % ArDrone
        A = J.mControl(A);                      % joystick command (priority)
        
        P = fCompensadorDinamico(P);
        
        % Get Performace Scores       
        NSBF.mFormationPerformace(sampleTime,toc(t),index);                                      
        
        % Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' NSBF.pPos.Qd' NSBF.pPos.Qtil' toc(t)]);
        fprintf(Arq,'\n\r');
        
        % Variable to feed plotResults function
        data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' NSBF.pPos.Qd' NSBF.pPos.Qtil' toc(t)];
        
        %% Send control signals to robots      
        
        
        
        % Send Ardrone control signals (only if in flying state)                        
%         P.rSendControlSignals; % Para o robo real o send msg é suficiente
        Rede.mSendMsg(P);       % send Pioneer data to network
        
        A.rSendControlSignals;
        
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
        A.mCADplot;                    % ArDrone
        
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

%% Close file and stop robot
fclose(Arq);

%% Send control signals
P.pSC.Ud = [0  ;  0];

for ii = 1:5
    Rede.mSendMsg(P);
end

%%
% % % % P.pSC.Ud = [-.2  ;  0];
% % % % 
% % % % for ii = 1:5
% % % %     Rede.mSendMsg(P);
% % % % end

%%

% Lan

% Land drone
if A.pFlag.Connected == 1
    A.rLand;                % Commando to Land Drone 
end


%% Plot results
% figure;
%plotResults(data);

% figure;
close all;
% NSB_PlotResults(data,1,"Defaut Controller");
% NSBF.mDisplayFormationPerformace("Defaut Controller");
 
NSB_PlotResults(data,1," - NSB - Formation Priority");
NSBF.mDisplayFormationPerformace(" - NSB - Formation Priority");
 
% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx




