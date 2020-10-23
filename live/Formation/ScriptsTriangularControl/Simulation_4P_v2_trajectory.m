%% Triagular Formation Pioneer
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  %
% Initial Comands
clear; close all; clc;
try
    fclose(instrfindall);
catch
end
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %
% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Classes
% Robots
n = 4;
for ii=1:n
    P{ii} = Pioneer3DX(ii);
    P{ii}.pFormation.ID = [];
    P{ii}.pFormation.nF = 0;
end

% Triangular Formation 3D
nF = n - 2;

TF{1} = TriangularFormationControl;
TF{2} = TriangularFormationControl;

TF{1}.pRobots  = [1 2 3];
TF{2}.pRobots  = [2 3 4];

P{1}.pFormation.ID = [1]; % Formação [a b c]
P{2}.pFormation.ID = [1 2];
P{3}.pFormation.ID = [1 2];
P{4}.pFormation.ID = [2];

% Ganhos           
TF{1}.pPar.K1 = 1*diag([    1.0    1.0    1.0    1.0   1.0   1.0   ]);     % kinematic control gain  - controls amplitude
TF{1}.pPar.K2 = 1*diag([    0.1    0.1    0.1    0.1   0.1   0.1   ]);     % kinematic control gain - control saturation

TF{2}.pPar.K1 = 1*diag([    1.0    1.0    1.0    1.0   1.0   1.0   ]);     % kinematic control gain  - controls amplitude
TF{2}.pPar.K2 = 1*diag([    0.1    0.1    0.1    0.1   0.1   0.1   ]);     % kinematic control gain - control saturation

% Create OptiTrack object and initialize
% OPT = OptiTrack;
% OPT.Initialize;

% Network
% Rede = NetDataShare;

% Método de Ponderação
method = 0;

%% Robots initial pose
% detect rigid body ID from optitrack
% idP{1} = getID(OPT,P{1});            % pioneer ID on optitrack
% idP{2} = getID(OPT,P{2})+1;          % pioneer ID on optitrack
% idP{3} = getID(OPT,P{3})+2;          % pioneer ID on optitrack
% 
% rb = OPT.RigidBody;                   % read optitrack data
% P{1} = getOptData(rb(idP{1}),P{1});   % get pioneer data
% P{2} = getOptData(rb(idP{2}),P{2});   % get pioneer data
% P{3} = getOptData(rb(idP{3}),P{3});   % get pioneer data

%% Variable initialization
data = [];

%% Formation initial parameters

distx = .5;
disty = .5;

P{1}.rSetPose([-distx -disty 0 0]');
P{2}.rSetPose([ distx -disty 0 0]');
P{3}.rSetPose([-distx  disty 0 0]');
P{4}.rSetPose([ distx  disty 0 0]');

% TFG.Xd = [ -distx distx -distx ; 
%            -disty -disty disty ; 
%             ones(1,3)];

TFG.Xd = [ -distx distx -distx distx; 
           -disty -disty disty disty; 
            ones(1,4)];

% TFG.Xd = [P{1}.pPos.X(1) P{2}.pPos.X(1) P{3}.pPos.X(1);
%             P{1}.pPos.X(2) P{2}.pPos.X(2) P{3}.pPos.X(2); 
%             ones(1,3)];

for ii=1:nF
    TF{ii}.pID = ii;
    TF{ii}.mIdentSeq(P);
    TF{ii}.mDirTrans(P);
%     TF{ii}.pPos.Xd = TF{ii}.pPos.X;
    TF{ii}.pPonderacao = method;
    TF{ii}.mInvTrans(P);
end

%Numero de formações que um robo pertence
for ii=1:nF
    P{ii}.pFormation.nF = P{ii}.pFormation.nF + 1;
    P{ii+1}.pFormation.nF = P{ii+1}.pFormation.nF + 1;
    P{ii+2}.pFormation.nF = P{ii+2}.pFormation.nF + 1;
end

%Variáveis extras para método de ponderação
for ii=1:n
    P{ii}.pPos.mXd = zeros(2,nF);
    P{ii}.pPos.mdXd = zeros(2,nF);
end

%% Plot

fig = figure(1);
axis([-4 4 -4 4]);
hold on;
grid on;

% Draw robots
for ii=1:n
    try
        P{ii}.mCADdel;
    catch
    end
end

P{1}.mCADplot(0.75,'b');
P{2}.mCADplot(0.75,'r');
P{3}.mCADplot(0.75,'g');
P{4}.mCADplot(0.75,'m');

drawnow;

%% Simulation

fprintf('\nStart..............\n\n');
% Time variables initialization
T_CONTROL = 0.030;
T_PIONEER = 0.030;
T_PLOT = inf;

rX = 0.75;       % [m]
rY = 1.5;       % [m]

T = 60;         % [s]
Tf = 120;       % [s]
w = 2*pi/T;     % [rad/s]

T_FP = 1;      % [s]

caso = 0;

t_control = tic;
t_Pioneer = tic;
t_plot = tic;

t_integ_1 = tic;
t_integ_2 = tic;

t  = tic;

while toc(t) < Tf

    if toc(t_control) > T_CONTROL
               
        t_control = tic;
            
% %         Rede.mReceiveMsg;
% %         if length(Rede.pMSG.getFrom)>2
% %             P1.pSC.U  = Rede.pMSG.getFrom{4}(29:30);       % current velocities (robot sensors)
% %             P1X       = Rede.pMSG.getFrom{4}(14+(1:12));   % current position (robot sensors)
% %             
% %             P2.pSC.U  = Rede.pMSG.getFrom{5}(29:30);       % current velocities (robot sensors)
% %             P2X       = Rede.pMSG.getFrom{5}(14+(1:12));   % current position (robot sensors)
% %             
% %             P3.pSC.U  = Rede.pMSG.getFrom{6}(29:30);       % current velocities (robot sensors)
% %             P3X       = Rede.pMSG.getFrom{6}(14+(1:12));   % current position (robot sensors)
% %         end
% %         
% %         % Get optitrack data
% %         rb = OPT.RigidBody;                   % read optitrack data
% %         P{1} = getOptData(rb(idP{1}),P{1});   % get pioneer data
% %         P{2} = getOptData(rb(idP{2}),P{2});   % get pioneer data
% %         P{3} = getOptData(rb(idP{3}),P{3});   % get pioneer data

        for ii=1:n
            P{ii}.rGetSensorData;    % Adquirir dados dos sensores - Pioneer
        end
        
        for ii=1:nF
            TF{ii}.mDirTrans(P);
        end
        
        %% Trajectory
        % Variáveis Formação Global
        t_traj = toc(t);
        a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
        tp = a*Tf;
        
        xG = rX*sin(w*tp);
        yG = rY*sin(2*w*tp);
        psiG = 0;

        H = [ cos(psiG), -sin(psiG), xG; 
              sin(psiG ), cos(psiG), yG; 
              0,          0,         0];
        
        TFG.Xr = H*TFG.Xd;
        
        for ii=1:nF
            TF{ii}.pPos.Xd(1:3) = TFG.Xr(:,ii);
            TF{ii}.pPos.Xd(4:6) = TFG.Xr(:,ii+1);
            TF{ii}.pPos.Xd(7:9) = TFG.Xr(:,ii+2);
        end
        for ii=1:nF
            TF{ii}.mDirTransDesired;
            TF{ii}.pPos.dQd(1) = w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp);          % dxF
            TF{ii}.pPos.dQd(2) = 2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp);      % dyF
        end
        
        for ii=1:nF
            TF{ii}.pPos.Qtil = TF{ii}.pPos.Qd - TF{ii}.pPos.Q;
            if( abs(TF{ii}.pPos.Qtil(3)) > pi)
                if(TF{ii}.pPos.Qtil(3) >= 0)
                    TF{ii}.pPos.Qtil(3) = -2*pi + TF{ii}.pPos.Qtil(3);
                else
                    TF{ii}.pPos.Qtil(3) = 2*pi + TF{ii}.pPos.Qtil(3);
                end
            end
        end    
        
%         LF1.mInvTrans;                                 % Transformada Inversa Qd --> Xd
%         LF2.mInvTrans;                                 % Transformada Inversa Qd --> Xd
%         
        %% Control
        
        % Formation Control      
        TF{1}.mTriangularFormationControl(toc(t_integ_1));
        t_integ_1 = tic;
        
        TF{2}.mTriangularFormationControl(toc(t_integ_2));
        t_integ_2 = tic;

        % Pioneer 
        TF{1}.mCSweighting(TF,P)
        
        % Controlador Dinâmico        
        % Ganhos pré-definidos
%         cgains = [ 0.35  0.35  0.80  0.80  0.75  0.75  0.12  0.035 ];
        cgains = [ 0.10  0.10  0.75  0.75  0.75  0.75  0.10  0.05 ];
        
        P{1} = fDynamicController(P{1},cgains);     % Pioneer Dynamic Compensator
        P{2} = fDynamicController(P{2},cgains);     % Pioneer Dynamic Compensator
        P{3} = fDynamicController(P{3},cgains);     % Pioneer Dynamic Compensator
        P{4} = fDynamicController(P{4},cgains);     % Pioneer Dynamic Compensator
        
        %% Save data
        % Variable to feed plotResults function
        data = [  data  ; P{1}.pPos.Xd'   P{1}.pPos.X'      P{1}.pSC.Ud(1:2)'  P{1}.pSC.U(1:2)' ...
                          P{2}.pPos.Xd'   P{2}.pPos.X'      P{2}.pSC.Ud(1:2)'  P{2}.pSC.U(1:2)' ...
                          P{3}.pPos.Xd'   P{3}.pPos.X'      P{3}.pSC.Ud(1:2)'  P{3}.pSC.U(1:2)' ...
                          P{4}.pPos.Xd'   P{4}.pPos.X'      P{4}.pSC.Ud(1:2)'  P{4}.pSC.U(1:2)' ...
                          TF{1}.pPos.Qd'  TF{1}.pPos.Q'     TF{1}.pPos.Qtil'   TF{1}.pPos.Xd'   ...
                          TF{2}.pPos.Qd'  TF{2}.pPos.Q'     TF{2}.pPos.Qtil'   TF{2}.pPos.Xd'   ...
                          toc(t)]; % TENTAR PRÉ-ALOCAR A MEMÓRIA
                      
            % %       %   1 -- 12         13 -- 24          25 -- 26           27 -- 28 
            % %           P1.pPos.Xd'     P1.pPos.X'        P1.pSC.Ud'         P1.pSC.U'
            % %           
            % %       %   29 -- 40        41 -- 52          53 -- 54           55 -- 56    
            % %           P2.pPos.Xd'     P2.pPos.X'        P2.pSC.Ud'         P2.pSC.U'
            % %
            % %       %   57 -- 68        69 -- 80          81 -- 82           83 -- 84    
            % %           P3.pPos.Xd'     P3.pPos.X'        P3.pSC.Ud'         P3.pSC.U'
            % %
            % %       %   85 -- 96        97 -- 108         109 -- 110         111 -- 112    
            % %           P4.pPos.Xd'     P4.pPos.X'        P4.pSC.Ud'         P4.pSC.U'
            % %             
            % %       %   113 -- 118      119 -- 124        125 -- 130         131 -- 139          
            % %           TF1.pPos.Qd'    TF1.pPos.Q'       TF1.pPos.Qtil'     TF1.pPos.Xd'    
            % %
            % %       %   140 -- 145      146 -- 151        152 -- 157         158 -- 166          
            % %           TF2.pPos.Qd'    TF2.pPos.Q'       TF2.pPos.Qtil'     TF2.pPos.Xd'    
            % %
            % %       %   167
            % %       %   toc(t)  ];

        %% Send control signals to robots
        
%         Rede.mSendMsg(P{1});
%         Rede.mSendMsg(P{2});
%         Rede.mSendMsg(P{3});
%         Rede.mSendMsg(P{4});

        P{1}.rSendControlSignals;
        P{2}.rSendControlSignals;
        P{3}.rSendControlSignals;
        P{4}.rSendControlSignals;

        % Draw robots
        if toc(t_plot) > T_PLOT
            t_plot = tic;
            try
                P{1}.mCADdel;
                P{2}.mCADdel;
                P{3}.mCADdel;
                P{4}.mCADdel;
            catch
            end

            P{1}.mCADplot(0.75,'b');
            P{2}.mCADplot(0.75,'r');
            P{3}.mCADplot(0.75,'g');
            P{4}.mCADplot(0.75,'m');
            drawnow;

            if toc(t_control) > 0.100
                disp(['Estourou o tempo: ',num2str(1000*(toc(t_control)-0.100))]); 
            end
        end
    end

end

fprintf('\nFim..............\n\n');

% % %% Send control signals
% % P{1}.pSC.Ud = -[.05  ;  0];
% % P{2}.pSC.Ud = -[.05  ;  0];
% % P{3}.pSC.Ud = -[.05  ;  0];
% % P{4}.pSC.Ud = -[.05  ;  0];
% % 
% % for ii = 1:100
% %     Rede.mSendMsg(P{1});
% %     Rede.mSendMsg(P{2});
% %     Rede.mSendMsg(P{3});
% % end
% % 
% % %% Send control signals
% % P1.pSC.Ud = [0  ;  0];
% % P2.pSC.Ud = [0  ;  0];
% % P3.pSC.Ud = [0  ;  0];
% %     
% % for ii = 1:100
% %     Rede.mSendMsg(P1);    
% %     Rede.mSendMsg(P2);
% %     Rede.mSendMsg(P3);   
% % end

%% Plot results
% % figure;
% plotResultsLineControl_v2(data);
% plotResultsLineControl(data);

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
