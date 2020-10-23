%% 3D Line Formation Pioneer-Drone
clear
close all
clc

try
    fclose(instrfindall);
catch
end

%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Configure simulation window
fig = figure(1);
axis([-8 8 -8 8])
pause(1)

%% Load Classes
% Robot class
% ...........................method A......................................
% RA = input('Digite o tipo do Robô (Pioneer3DX(ID)||ArDrone(ID)):  ');
% formation = input('Defina o tipo de formação (TT||TA||AT||AA): ');
% switch formation
%     case 'TT'
%         RB = Pioneer3DX;
%     case 'TA'
%         RB = ArDrone;
%     case 'AT'
%         RB = Pioneer3DX;
%     case 'AA'
%         RB = ArDrone;
%     otherwise
%         disp('Unknown formation configuration!');
% end
% ...................method B...............................
% ID = input('Digite o ID do robô:  ');
% RA = Pioneer3DX(ID);
% RB = Pioneer3DX;
% ....................method C................................
% Create the second robot of formation
% ID 1 = Pioneer ::: ID 2 = ArDrone

ID = input('Digite o ID do robô:  ');
if ID == 1
    RA = Pioneer3DX(ID);
    RB = ArDrone;
elseif ID == 2
    RA = ArDrone(ID);
    RB = Pioneer3DX;
end

% Communication class
Rede = NetDataShare;

% Formation class
LF = LineFormation3D;

%% Open file to save data
%Only robot 1 saves data
if RA.pID==1
    NomeArq = datestr(now,30);
    cd('DataFiles')
    cd('Log_FormacaoLinha3D')
    Arq = fopen(['FL3d_' NomeArq '.txt'],'w');
    cd(PastaAtual)
end

%% Robot/Simulator conection
% RA.rConnect;

%% Robots initial pose
if RA.pPar.Model(1)=='P'
    
    % Xo = input('Type initial pose([x y z psi]): ');
    % A.pPos.X([1 2 6]) = [Xo(1) ; Xo(2); Xo(3)];
    
    Xo = [0 1 0 0];
    RA.rSetPose(Xo');
end

shg              % show graphic window
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

% U = [];
% ke = 0;
% kr = 0;
acc = [];   % salva aceleração calculada
cont = 0;
cont2 = 0;

% Desired formation [xf yf zf rhof alfaf betaf]
% xsq = 1.2;
% ysq = 1.24;
% zsq = 0;
% LF.pPos.Qd = [3*xsq 3*ysq xsq 0]';
LF.pPos.Qd = [3 2 0 1 deg2rad(90) deg2rad(30)]';

%% Formation initial error
% Get data from other robot
if length(Rede.pMSG.getFrom) >= 1
    %  ID = 2
    if RA.pID > 1
        
        % Get first robot data
        if ~isempty(Rede.pMSG.getFrom{1})
            
            Xa = Rede.pMSG.getFrom{1}(14+(1:3));  % robot 1
            Xb = RA.pPos.X((1:3));                % robot 2
            
            % Formation initial pose
            LF.pPos.X = [Xa; Xb];
            
        end
        % ID = 1
    else
        % caso haja dados dos dois robôs na rede
        if length(Rede.pMSG.getFrom) == RA.pID+1
            
            Xa = RA.pPos.X(1:3);                         % robot 1
            Xb = Rede.pMSG.getFrom{RA.pID+1}(14+(1:3));  % robot 2
            
            % Formation initial pose
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

% Time variables initialization
t = tic;
tc = tic;
tp = tic;
timeout = 120;   % maximum simulation duration

while abs(LF.pPos.Qtil(1))>erroMax(1) || abs(LF.pPos.Qtil(2))>erroMax(2) || ...
        abs(LF.pPos.Qtil(3))>erroMax(3) || abs(LF.pPos.Qtil(4))>erroMax(4)|| ...
        abs(LF.pPos.Qtil(5))>erroMax(5) || abs(LF.pPos.Qtil(6))>erroMax(6) % formation errors
    
    if toc(tc) > 0.1
        
        tcc = tic;
        
        RA.rGetSensorData;
        
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
                    
                    Xa = Rede.pMSG.getFrom{1}(14+(1:3));    % robot 1
                    Xb = RA.pPos.X(1:3);                    % robot 2
                    
                    % Robot Position
                    LF.pPos.X = [Xa; Xb];
                    
                    % Formation Controller
                    LF.mFormationControl;
                    
                    
                   RA.pPos.Xd(1:3) = RA.pPos.Xd(1:3) + LF.pPos.dXr(4:6)*0.1;
                   RA.pPos.Xd(7:9) = LF.pPos.dXr(4:6);
%                     
%                     % Desired Position
%                     LF.mInvTrans;
%                     RA.pPos.Xd(1:3) = LF.pPos.Xr(4:6);
%                     
%                     % Robot Control Velocity
%                     RA.sInvKinematicModel(LF.pPos.dXr(4:6));
                    
                    % Assign received data
                    RB.pPos.Xd = Rede.pMSG.getFrom{1}(2+(1:12));
                    RB.pPos.X  = Rede.pMSG.getFrom{1}(14+(1:12));
                    RB.pSC.Ud  = Rede.pMSG.getFrom{1}(26+(1:4));
                    RB.pSC.U   = Rede.pMSG.getFrom{1}(30+(1:4));
                    
                    % Other robot center pose
                    if RB.pPar.Model(1)=='P'
                        RB.pPos.Xc([1 2 6]) = RB.pPos.X([1 2 6]) + ...
                            [RB.pPar.a*cos(RB.pPos.X(6)); RB.pPar.a*sin(RB.pPos.X(6)); 0];
                    end
                    
                    if RA.pID==1   % only robot 1 saves data
                        % Save data (.txt file)
                        fprintf(Arq,'%6.6f\t',[RA.pPos.Xd' RA.pPos.X' RA.pSC.Ud' RA.pSC.U' ...
                            RB.pPos.Xd' RB.pPos.X' RB.pSC.Ud' RB.pSC.U' toc(t)]);
                        fprintf(Arq,'\n\r');
                    end
                end
                
                %% ID = 1 .................................................................
            else
                
                if length(Rede.pMSG.getFrom) == RA.pID+1   % caso haja dados dos dois robôs na rede
                    
                    Xa = RA.pPos.X(1:3);                         % robot 1
                    Xb = Rede.pMSG.getFrom{RA.pID+1}(14+(1:3));  % robot 2
                    
                    % Robot position
                    LF.pPos.X = [Xa; Xb];
                    
                    % Formation Controller
                    LF.mFormationControl;
                    
                    % Desired position
                    LF.mInvTrans;
                    RA.pPos.Xd(1:3) = LF.pPos.Xr(1:3);
                    
                    % Robot Control Velocity
                    RA.sInvKinematicModel(LF.pPos.dXr(1:3));   % Calculate control signals
                    
                    % Assign received data
                    RB.pPos.Xd = Rede.pMSG.getFrom{RA.pID+1}(2+(1:12));
                    RB.pPos.X  = Rede.pMSG.getFrom{RA.pID+1}(14+(1:12));
                    RB.pSC.Ud  = Rede.pMSG.getFrom{RA.pID+1}(26+(1:4));
                    RB.pSC.U   = Rede.pMSG.getFrom{RA.pID+1}(30+(1:4));
                    
                    % Robot center pose (for pioneer only)
                    if RB.pPar.Model(1)=='P'
                        RB.pPos.Xc([1 2 6]) = RB.pPos.X([1 2 6]) + ...
                            [RB.pPar.a*cos(RB.pPos.X(6)); RB.pPar.a*sin(RB.pPos.X(6)); 0];
                    end
                    %%
                    if length(RA.pSC.Ud)==2
                        cont = cont+1;
                        RA.pSC.Ud = [RA.pSC.Ud;0;0];
                    end
                    
                    
                    if length(RA.pSC.U)==2
                        cont2 = cont2+1;
                        RA.pSC.U = [RA.pSC.U;0;0];
                    end
                    
                    %%
                    if RA.pID==1   % only robot 1 saves data
                        % Save data (.txt file)
                        fprintf(Arq,'%6.6f\t',[RA.pPos.Xd' RA.pPos.X' RA.pSC.Ud' RA.pSC.U' ...
                            RB.pPos.Xd' RB.pPos.X' RB.pSC.Ud' RB.pSC.U' toc(t)]);
                        fprintf(Arq,'\n\r');
                    end
                end
            end
        end
        
        %         U = [U [A.pSC.U; B.pSC.U]];  % velocidade dos robôs
        
        
        %% Dynamic compensation
        % Dynamic Compensator
        if RA.pPar.Model(1)=='P'
            %             %% Gambiarrinha para padronizar o tamanho do vetor ........
            %             RA.pSC.U = [0;0;0;0];
            %             % .........................................................
            
            %             RA = fCompensadorDinamico(RA);
            
            %         % ---------------------------------------
            %         % If not using dynamic compensation
                    RA.pSC.Ud = RA.pSC.Ur;
            %         % --------------------------------------
            % Send Control Signal
            RA.rSendControlSignals;
            
            %% Gambiarrinha para padronizar o tamanho do vetor ........
%             RA.pSC.U = [0;0;0;0];
            % .........................................................
            
        elseif RA.pPar.Model(1)=='A'
            
          
            % Dynamic Controller
            RA = cUnderActuatedController(RA);
            % ---------------------------------------
            % If not using dynamic compensation
%             RA.pSC.Ud = RA.pSC.Ur;
            % --------------------------------------
            
            % Send Control Signal
            RA.rSendControlSignals;
            
        end
        
        % Send message to network
        Rede.mSendMsg(RA);
        %         ke = ke + 1;
        
        % Draw robot
        if toc(tp) > 0.1
            tp = tic;
            
            %             RA.mCADdel
            %             RB.mCADdel
            %
            if RA.pPar.Model(1)=='P'
                RA.mCADdel
                RA.mCADplot2D('b')
                RB.mCADplot
            elseif RA.pPar.Model(1) == 'A'
                RA.mCADplot
                RB.mCADdel
                RB.mCADplot2D('b')
            end
            
            grid on
            drawnow
        end
        
        % Simulation timeout
        if toc(t)> timeout
            disp('Too much time has passed already. I guess we should stop...');
            break
        end
        
    end
end
%% Close file and stop robot
if RA.pID==1
    fclose(Arq);
end

% Send control signals to robot
% RA.sInvKinematicModel(zeros(3,1));
% RA.pSC.Ud = RA.pSC.Ur;
% RA.rSendControlSignals;

% A.mDisconnect;


