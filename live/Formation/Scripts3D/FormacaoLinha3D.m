clear
close all
clc

try
    fclose(instrfindall)
end

%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_FormacaoLinha3D')
Arq = fopen(['FL3d_' NomeArq '.txt'],'w');
cd(PastaAtual)

%% Configure simulation window
fig = figure(1);
axis([-8 8 -8 8])
pause(1)

%% Load Classes
% Robot class
nRobos = 2;
% % A = input('Digite a classe do Robô (Ex. Pioneer3DX(1)):  ');
% % formation = input('Defina o tipo de formação (TT||TA||AT||AA)');
% % switch formation
% %     case TT
% %         B = Pioneer3DX;
% %     case TA
% %         B = ArDrone;
% %     case AT
% %         B = Pioneer3DX;
% %     case AA
% %         B = ArDrone;
% %     otherwise
% %         disp('Unknown formation configuration!');
% % end

ID = input('Digite o ID do robô:  ');
RA = Pioneer3DX(ID);
RB = Pioneer3DX;

% Communication class
Rede = NetDataShare;

% Formation class
LF = LineFormation3D;

%% Robots initial pose
Xo = input('Type initial pose([x y z psi]): ');

% Xo = [0 -1.25 0];
% A.pPos.X([1 2 6]) = [Xo(1) ; Xo(2); Xo(3)];

%% Robot/Simulator conection
RA.rConnect;
RA.rSetPose(Xo');
shg
pause(2)
disp('Start..............');

%% Establish first communication
tm = tic;
% Send and read data
while isempty(Rede.pMSG.getFrom)
    Rede.mSendMsg(RA);
    if toc(tm) > 0.1
        tm = tic;
        Rede.mReceiveMsg;
        
    end
end
disp('Communication established .....');

%% Variable initialization
Xa = RA.pPos.X(1:6);

U = [];
ke = 0;
kr = 0;
acc = [];   % salva aceleração calculada

% Desired formation [xf yf zf rhof alfaf betaf]
% xsq = 1.2;
% ysq = 1.24;
% zsq = 0;
% LF.pPos.Qd = [3*xsq 3*ysq xsq 0]';
LF.pPos.Qd = [4 2 0 1 deg2rad(0) deg2rad(0)]';

%% Initial error calculation
% Get data from other robot
if length(Rede.pMSG.getFrom) >= 1
    %  ID = 2
    if RA.pID > 1
        
        % Get first robot data
        if ~isempty(Rede.pMSG.getFrom{1})
            
            Xa = Rede.pMSG.getFrom{1}(14+(1:3));  % robot 1
            Xb = RA.pPos.X((1:3));                % robot 2
            
            % Initial pose
            LF.pPos.X = [Xa; Xb];
            
        end
    % ID = 1
    else
        
        if length(Rede.pMSG.getFrom) == RA.pID+1   % caso haja dados dos dois robôs na rede
            
            Xa = RA.pPos.X(1:3);                         % robot 1
            Xb = Rede.pMSG.getFrom{RA.pID+1}(14+(1:3));  % robot 2
            
            % Initial pose
            LF.pPos.X = [Xa; Xb];
            
        end
    end
end

% Formation initial pose
LF.mDirTrans;

% Formation Error
LF.mFormationError;

%% Simulation
% Maximum error permitted
erroMax = [.2 .2 .2 .2 deg2rad(5) deg2rad(5)]; 

% Time variables
t = tic;
tc = tic;
tp = tic;
timeout = 120;   % maximum simulation duration

while abs(LF.pPos.Qtil(1))>erroMax(1) || abs(LF.pPos.Qtil(2))>erroMax(2) || ...
        abs(LF.pPos.Qtil(3))>erroMax(3) || abs(LF.pPos.Qtil(4))>erroMax(4)|| ...
        abs(LF.pPos.Qtil(5))>erroMax(5) || abs(LF.pPos.Qtil(6))>erroMax(6) % formation errors
        
    if toc(tc) > 0.1
        
        tcc = tic;
        RA.rGetSensorData
        
        % Read network data
        Rede.mReceiveMsg;
        
        % Buscar SEMPRE robô com ID+1 para formação
        % Variável robôs da rede
        % Verifica se há mensagem
        %% Formation control
        if length(Rede.pMSG.getFrom) >= 1
            %%  ID = 2 ..................................................................
            if RA.pID > 1
               
                % pegar primeiro robo
                if ~isempty(Rede.pMSG.getFrom{1})
                    %                     kr = kr + 1;
                    %                     disp([ke kr])
                    
                    Xa = Rede.pMSG.getFrom{1}(14+(1:3));    % robô 1
                    Xb = RA.pPos.X(1:3);                     % robô 2
                    
                    % Robot Position
                    LF.pPos.X = [Xa; Xb];
                    
                    % Formation Controller
                    LF.mFormationControl;
                    
                    % Robot Control Velocity
                    RA.sInvKinematicModel(LF.pPos.dXr(4:6));    % Calculate control signals
                    
                    % Desired Position
                    LF.mInvTrans;
                    RA.pPos.Xd(1:3) = LF.pPos.Xr(4:6);
                    
                    % % Atribui sinal de controle ao robô
                    % A.pSC.Ud = LF.pSC.Ud(3:4);
                    
                    % Atribuindo valores desejados do controle de formação
                    RB.pPos.Xd = Rede.pMSG.getFrom{1}(2+(1:12));
                    RB.pPos.X  = Rede.pMSG.getFrom{1}(14+(1:12));
                    RB.pSC.Ur  = Rede.pMSG.getFrom{1}(26+(1:2));
                    RB.pSC.U   = Rede.pMSG.getFrom{1}(28+(1:2));
                    
                    % Postura do centro do robô
                    RB.pPos.Xc([1 2 6]) = RB.pPos.X([1 2 6]) + ...
                        [RB.pPar.a*cos(RB.pPos.X(6)); RB.pPar.a*sin(RB.pPos.X(6)); 0];
                    
                    % Save data (.txt file)
                    fprintf(Arq,'%6.6f\t',[RA.pPos.Xd' RA.pPos.X' RA.pSC.Ur' RA.pSC.U' ...
                        RB.pPos.Xd' RB.pPos.X' RB.pSC.Ur' RB.pSC.U' toc(t)]);
                    fprintf(Arq,'\n\r');
                end
                
                %% ID = 1 .................................................................
            else
                
                if length(Rede.pMSG.getFrom) == RA.pID+1   % caso haja dados dos dois robôs na rede
                    
                    Xa = RA.pPos.X(1:3);                         % robô 1
                    Xb = Rede.pMSG.getFrom{RA.pID+1}(14+(1:3));  % robô 2
                    
                    % Robot position
                    LF.pPos.X = [Xa; Xb];
                    
                    % Formation Controller
                    LF.mFormationControl;
                    
                    % Robot Control Velocity
                    RA.sInvKinematicModel(LF.pPos.dXr(1:3));   % Calculate control signals
                    
                    % Desired position
                    LF.mInvTrans;
                    RA.pPos.Xd(1:3) = LF.pPos.Xr(1:3);
                    
                    %                     % Atribui sinal de controle
                    %                     A.pSC.Ud = LF.pSC.Ud(1:2);
                    %
                    % Atribuindo valores desejados do controle de formação
                    RB.pPos.Xd = Rede.pMSG.getFrom{RA.pID+1}(2+(1:12));
                    RB.pPos.X  = Rede.pMSG.getFrom{RA.pID+1}(14+(1:12));
                    RB.pSC.Ur  = Rede.pMSG.getFrom{RA.pID+1}(26+(1:2));
                    RB.pSC.U   = Rede.pMSG.getFrom{RA.pID+1}(28+(1:2));
                    
                    % Robot center pose
                    RB.pPos.Xc([1 2 6]) = RB.pPos.X([1 2 6]) + ...
                        [RB.pPar.a*cos(RB.pPos.X(6)); RB.pPar.a*sin(RB.pPos.X(6)); 0];
                    
                    % Save data (.txt file)
                    fprintf(Arq,'%6.6f\t',[RA.pPos.Xd' RA.pPos.X' RA.pSC.Ur' RA.pSC.U' ...
                        RB.pPos.Xd' RB.pPos.X' RB.pSC.Ur' RB.pSC.U' toc(t)]);
                    fprintf(Arq,'\n\r');
                end
            end
        end
        
        %         U = [U [A.pSC.U; B.pSC.U]];  % velocidade dos robôs
        %
%                 disp('UrA');
%                 display(RA.pSC.Ud);
%         
%                 disp('UrB');
%                 display(RB.pSC.Ud);
%         
        
        %% Dynamic compensation
        
        % Dynamic Compensator
        RA = fCompensadorDinamico(RA);
%         
%         % ---------------------------------------
%         % If not using dynamic compensation
%         RA.pSC.Ud = RA.pSC.Ur;
%         % --------------------------------------
        %% Send Control Signal
        RA.rSendControlSignals;
        
        % Send message to network
        Rede.mSendMsg(RA);
        %         ke = ke + 1;
        
    end
    % Draw robot
    if toc(tp) > 0.1
        tp = tic;
        
        RA.mCADdel
        RB.mCADdel
        
        RA.mCADplot2D('b')
        RB.mCADplot2D('r')
        
        grid on
        drawnow
    end
    
    % Simulation timeout
    if toc(t)> timeout
        disp('Too much time has passed already. Let s finish this...');
        break
    end
    
end

%% Close file and stop robot
fclose(Arq);

% Send control signals to robot
RA.sInvKinematicModel(zeros(3,1));
RA.pSC.Ud = RA.pSC.Ur;
RA.rSendControlSignals;

% A.mDisconnect;


