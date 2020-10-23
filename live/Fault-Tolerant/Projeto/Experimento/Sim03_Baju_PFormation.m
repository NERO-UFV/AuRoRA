clear all
close all
clc

try
    fclose(instrfindall);
catch
end

%% Load Class
% try
%     % Load Classes
%     RI = RosInterface;
%     setenv('ROS_IP','192.168.0.158')
%     setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
%     RI.rConnect('192.168.0.146');
%     
%     % Create OptiTrack object and initialize
%     OPT = OptiTrack;
%     OPT.Initialize;
%     
%     P = RPioneer(1,'P1');
%     % detect rigid body ID from optitrack
%     idP = 1;
%     
%     % Joystick
%     J = JoyControl;
%     Joy = 1;
%     
%     disp('################### Load Class Success #######################');
%     
% catch ME
%     disp(' ');
%     disp(' ################### Load Class Issues #######################');
%     disp(' ');
%     disp(' ');
%     disp(ME);
%     
%     RI.rDisconnect;
%     rosshutdown;
%     return;
%     
% end
nP = 4;
for ii = 1:nP
    P{ii} = Pioneer3DX(ii);
    P{ii}.pPar.a = 0;
    P{ii}.pPar.alpha = 0;
end
% rb = OPT.RigidBody;          % read optitrack data
% P = getOptData(rb(idP),P);   % get pioneer data
%%
nF = 1;
for ii = 1:nF
    TF{ii} = TriangularFormationBaju(ii);
end



%% A
A = zeros(nP);
for ii = 1:nP
    for jj = 1:nP
        if jj == 3 && ii ~= 3
            A(ii,jj) = 1;
        end
    end
end

%%
% POSI플O INICIAL DA FORMA플O
Xfi = 0;
Yfi = 0;
Zfi = 0;

% ORIENTA플O INICIAL DA FORMA플O
THETAfi = 0;
PHIfi   = -pi/2;
PSIfi   = pi/2;

% POSTURA INICIAL DA FORMA플O
Pfi = 2;
Qfi = 2;
BETAfi = pi/3;

% ATRIBUINDO PARA FORMA플O
TF{1}.pPos.Q = [Xfi; Yfi; Zfi;
                THETAfi; PHIfi; PSIfi;
                Pfi; Qfi; BETAfi];

TF{1}.pPos.Qd = TF{1}.pPos.Q; 

% TRANSFORMADA INVERSA
TF{1}.tInvTrans;
TF{1}.pPos.X = TF{1}.pPos.Xd;
TF{1}.tDirTrans;
TF{1}.pPar.R = {1 2 3; 'P' 'P' 'P'};

P{1}.rSetPose([1 1 0 0]);
P{2}.rSetPose([-1 -1 0 0]);
P{3}.rSetPose([0 0 0 0]);
P{4}.rSetPose([0 0 0 0]);

%% CONTROLLER GAINS
pgains = [0.1 0.1 1];

%% TEMPORIZADORES

T_MAX = 100;
T_AMOSTRAGEM = 0.1;
T_PLOT = 0.11;

W = 2*pi/T_MAX;
rx = 1;
ry = 1;
Xd = [0; 0; 0];
SOMA = 1;

for ii = 1:nP
    DADOS{ii} = [];
end

T = tic;
Ta = tic;
Tp = tic;
D = tic;

COLOR = {'r','g','b'};
%% PLOT

figure
subplot(1,2,1)
P{1}.mCADplot(1,'r');
hold on
grid on
P{2}.mCADplot(1,'g');
P{3}.mCADplot(1,'b');
axis equal
axis([-2 2 -2 2])
subplot(1,2,2)
grid on
hold on
ylim([-2 2])
pause

while SOMA > 0.05
if toc(Ta) > T_AMOSTRAGEM 
    Ta = tic;
%     XdA = Xd;
%   Lemniscata (8')
%     Xd = [rx*sin(2*W*toc(T)); ry*sin(W*toc(T)); 0];

%   Circunferencia
%     Xd = [rx*cos(W*toc(T)); ry*sin(W*toc(T)); 0];
    
%   Posi豫o
%     Xd = [1; 1.5; 0];

%   Velocidade desejada
%     dXd = (Xd-XdA)/toc(D);
%     D = tic;
    
%   C
    for ii = 1:nP
        dXd{ii} = zeros(nP,1);
        for jj = 1:nP
            dXd{ii} = dXd{ii} + A(ii,jj).*(P{jj}.pPos.X(1:3) - P{ii}.pPos.X(1:3)); 
        end
    end
    
    

    for ii = 1:nP
        P{ii}.rGetSensorData;
        
        P{ii}.pPos.Xd = P{ii}.pPos.X;
        P{ii}.pPos.Xd(7:9) = dXd{ii};
        
        P{ii} = fKinematicControllerExtended(P{ii},pgains);        % new controller (by timotiu 2020)
        
        DADOS{ii}(end+1,:) = [P{ii}.pPos.Xd' P{ii}.pPos.X' toc(T)];
        
        P{ii}.rSendControlSignals;
    end
    SOMA = sqrt((DADOS{1}(end,13)-DADOS{3}(end,13))^2+(DADOS{1}(end,14)-DADOS{3}(end,14))^2+(DADOS{1}(end,15)-DADOS{3}(end,15))^2)+...
        sqrt((DADOS{2}(end,13)-DADOS{3}(end,13))^2+(DADOS{2}(end,14)-DADOS{3}(end,14))^2+(DADOS{2}(end,15)-DADOS{3}(end,15))^2);
end
if toc(Tp) > T_PLOT
    subplot(1,2,1)
    try
        P{1}.mCADdel;
        P{2}.mCADdel;
        P{3}.mCADdel;
%         delete(H)
    end
    P{1}.mCADplot(1,'r');
    P{2}.mCADplot(1,'g');
    P{3}.mCADplot(1,'b');
%     H(1) = plot(DADOS(:,1),DADOS(:,2),'--r');
%     H(2) = plot(DADOS(:,13),DADOS(:,14),'-k');
    subplot(1,2,2)
    try
        delete(H)
    end
    H(1) = plot(DADOS{1}(:,end),sqrt((DADOS{1}(:,13)-DADOS{3}(:,13)).^2+(DADOS{1}(:,14)-DADOS{3}(:,14)).^2+(DADOS{1}(:,15)-DADOS{3}(:,15)).^2),'r');
    H(2) = plot(DADOS{2}(:,end),sqrt((DADOS{2}(:,13)-DADOS{3}(:,13)).^2+(DADOS{2}(:,14)-DADOS{3}(:,14)).^2+(DADOS{2}(:,15)-DADOS{3}(:,15)).^2),'r');
    drawnow
end
end




















