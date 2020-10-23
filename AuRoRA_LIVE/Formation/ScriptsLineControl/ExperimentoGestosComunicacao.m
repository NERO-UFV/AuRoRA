%% 3D Line Formation Pioneer-Drone
% Pioneer is the reference of the formation
% The formation variables are:
% Q = [xf yf zf rhof alfaf betaf]
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  %
% Initial Comands

clear; close all; clc;
imaqreset;

pause
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
P = Pioneer3DX(1);

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Network
Rede = NetDataShare;
%% Inicialização Kinect
addpath('Mex');

k2 = Kin2('color','depth','body');
%% Importando rede Neural

AAA=load('RedeCompletaTKBLJ');
net=AAA.net;
%% Variáveis Kinect
Juntas=[5:12];
TempoGesto = 1.7;
intervalo = 0.035;
c = 150;
skeleton = [];
% Flag:
% 0 - Mão não foi aberta
% 1 - Mão foi aberta, pode classificar
flag = 0;
%% Identificação da Ações

NomesGestos = ['Gesto A - Tchau com mão direita','Gesto B - Itau',...
    'Gesto C - Águia média com mão direita', 'Gesto D - T médio com mão direita',...
    'Gesto E - Tchau com mão esquerda', 'Gesto F - Power rangers',...
    'Gesto G - T médio com mão esquerda','Gesto H - Águia média com mão esquerda',...
    'Gesto I - Pare','Gesto J - Sirva-me','Gesto K - Vem','Fica Parado','Gira','Andando'];
Indicies = [1,31,32,45,46,82,116,147,148,170,205,242,243,256,257,274,275,287,288,298,303,309];
h = figure;
pause
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
            disp('Waiting for message 1......')
            
        else
            break
        end
    end
end

%% Robots initial pose

% Xo = [0 0 0 0];
% P.rSetPose(Xo);    % define pose do robô
% P.pPos.X(1:2) = [0 0];

idP = getID(OPT,P);            % pioneer ID on optitrack
rb = OPT.RigidBody;            % read optitrack data
% P = getOptData(rb(idP),P);    % get pioneer data

%% Variable initialization

data = [];


%% Simulation

fprintf('\nStart..............\n\n');

%% Time variables initialization

T_CONTROL = 0.030;
T_PIONEER = 0.030;
T_PLOT = 0.200;
T_CAPTURA=0.035;

t_captura=tic;
t_control = tic;
t_plot = tic;
t_Pioneer = tic;        

t  = tic;

while toc(t) < 20000
%% Reconhecimento de Gestos
        if toc(t_captura)>T_CAPTURA
            t_captura=tic;
            validData = k2.updateData;
           
            while validData ~= 1
                validData = k2.updateData;
            end
            if validData
                
                [bodies, fcp, timeStamp] = k2.getBodies('Quat');
                numBodies = size(bodies,2);
                if numBodies > 0
                    %% Extraindo juntas:

                    skeletonJoints=bodies.Position';
                    %% Plotar as juntas

                    try
                        delete(h)
                    end
                    h = plot(skeletonJoints(:,1), skeletonJoints(:,2),'.','MarkerSize',15); % Plota no imshow as juntas
                    axis([-2.1 2.1 -2.1 2.1])
                    drawnow;

                    %% Teste de gatilho

    %                 Gatilho:
    %                 Se a mão esquerda ou a direita estiver fechada, classifica
    %                 0 unknown
    %                 1 not tracked
    %                 2 open
    %                 3 closed
    %                 4 lasso
    %                 Teste de confiança:
    %                 HandLeftConfidence, HandRightConfidence
    %                 0 low
    %                 1 high
                    %% Verifica se as juntas da mão são confiáveis e muda a flag de gatilho

                    if sum(bodies.TrackingState([22:25]))==8% Verifica se as mãos estão confiáveis
                        if bodies.LeftHandState== 2 ||  bodies.RightHandState == 2% Verifica se a mao ta aberta
                           flag = 1;
                        end
                    end
                    %% Verifica a flag e armazena o gesto

                    if flag == 1
                       skeleton = [skeleton skeletonJoints];                   
                       if size(skeleton,2) == 150
                           %% Refina a entrada
                           Gestos = skeleton(Juntas,:);
                           Hip_Center_Gesto = Gestos(1,:); % Pega o x,y,z dos esqueletos
                           Gestos = Gestos - (ones(length(Juntas),1) * Hip_Center_Gesto); % Centraliza
                           Gestos_Quadrado = Gestos*Gestos'; % Transforma [Gestos] em quadrada, multiplica ela pela transposta
                           AVSample = eig(Gestos_Quadrado);
                           %% Classifica
                           Saida = net(AVSample);
                           [~,pos] = max(Saida);                     
                           flag = 0; % Retorna flag ao estado normal
                           skeleton=[]; % Reseta o esqueleto
                           Classificado = [NomesGestos(Indicies(2*pos-1):Indicies(2*pos))];
                           disp(Classificado)
                           if pos==1
                               P.pSC.Ud = [0  ;  0.5];
                               
                               for ii = 1:50
                                   Rede.mSendMsg(P);
                               end
                           end
                           if pos==2
                               P.pSC.Ud = [0.1  ;  0];
                               
                               for ii = 1:50
                                   Rede.mSendMsg(P);
                               end
                           end
                           if pos==3
                               P.pSC.Ud = [0  ;  -0.5];
                               
                               for ii = 1:50
                                   Rede.mSendMsg(P);
                               end
                           end
                           if pos==4
                               P.pSC.Ud = [0  ;  0];
                               
                               for ii = 1:50
                                   Rede.mSendMsg(P);
                               end
                           end
                           if pos==5
                               P.pSC.Ud = [-0.2  ;  0];
                               
                               for ii = 1:50
                                   Rede.mSendMsg(P);
                               end
                           end
                       end
                    end
                end
            end
        end
        
%     if toc(t_control) > T_CONTROL      
%         %% Get data from robot and Optitrack    
%         t_control = tic;  
%         Rede.mReceiveMsg;
%         if length(Rede.pMSG.getFrom)>1
%             P.pSC.U  = Rede.pMSG.getFrom{2}(29:30);       % current velocities (robot sensors)
%             PX       = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
%         end
%         rb = OPT.RigidBody;             % read optitrack
%         P = getOptData(rb(idP),P);
%        
%         %% Control
%         if norm(P.pPos.Xtil(1:2))<0.1         
%             P.pSC.Ud = [0; 0];
%             P.pPos.Xd(1:2)=ginput(1)';
%         end        
%        
%         %% Save data        
%         % Variable to feed plotResults function
%         data = [  data  ; P.pPos.Xd'     P.pPos.X'        P.pSC.Ud(1:2)'    P.pSC.U(1:2)' ...
%                           toc(t)]; 
%             % %       %   1 -- 12         13 -- 24          25 -- 26           27 -- 28 
%             % %       %   P1.pPos.Xd'     P1.pPos.X'        P1.pSC.Ud'         P1.pSC.U'
%             % %
%             % %       %   29
%             % %       %   toc(t)  ];
% 
%                
%         %% Send control signals to robots
%         
%         Rede.mSendMsg(P);      
%         %         P.rSendControlSignals;
%                
%     end

end

%% Send control signals
P.pSC.Ud = -[.1  ;  0];

for ii = 1:50
    Rede.mSendMsg(P);
end

%% Send control signals
P.pSC.Ud = [0  ;  0];
    
for ii = 1:50
    Rede.mSendMsg(P);     
end

