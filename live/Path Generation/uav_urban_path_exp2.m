%% Simulação do planejamento de trajetória para bambolês (wayposes)
% - Implementar seguimento de caminho
% - Analisar velocidade/curvatura no tempo, se não tiver ok corrigir
% - Distorcer curva para proteger bambolês passados

% Boas práticas
close all
clear
clc

try
    fclose(instrfindall)
    % Rotina para buscar pasta raiz
    PastaAtual = pwd;
    PastaRaiz = 'AuRoRA 2018';
    cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
    addpath(genpath(pwd))
    
    disp('Diretório raiz e caminho bem definidos!')
catch
    disp('Diretório raiz errado!')
end

%% Definição da cena: ===================================================== 
% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
A = ArDrone(1);
B1 = ArDrone(2);
B2 = ArDrone(3);


% detect rigid body ID from optitrack
idA = getID(OPT,ArDrone,1);     % ID do bambolê no optitrack
idB1 = getID(OPT,ArDrone,2);     % ID do bambolê 1 no optitrack
idB2 = getID(OPT,ArDrone,3);     % ID do bambolê 2 no optitrack

rb = OPT.RigidBody;          % read optitrack data

B1 = getOptData(rb(idB1),B1);   % get hulla-hoop 1 data
B2 = getOptData(rb(idB2),B2);   % get hulla-hoop 1 data

% Orientação:
Rz1 = [cos(B1.pPos.X(6)) -sin(B1.pPos.X(6)) 0;...
       sin(B1.pPos.X(6)) cos(B1.pPos.X(6)) 0; 0 0 1];
   
Rn = [cos((90)*(pi/180)) -sin((90)*(pi/180)) 0;...
      sin((90)*(pi/180)) cos((90)*(pi/180)) 0; 0 0 1];

Rz2 = [cos(B2.pPos.X(6)) -sin(B2.pPos.X(6)) 0;...
       sin(B2.pPos.X(6)) cos(B2.pPos.X(6)) 0; 0 0 1];

% Bambolê 1:
b1 = B1.pPos.X(1:3)'; 
n1 = Rz1*[1;0;0]; n1 = Rn*n1;
v1 = B1.pPos.X(1:3)+[0;0.63;0]; v1 = v1./norm(v1); 
u1 = B1.pPos.X(1:3)+[0;0;.63];  u1 = u1./norm(u1);

% Bambolê 2:
b2 = B2.pPos.X(1:3)'; 
n2 = Rz2*[1;0;0]; n2 = -Rn*n2;
v2 = B2.pPos.X(1:3)+[0;0.63;0]; v2 = v2./norm(v2);  
u2 = B2.pPos.X(1:3)+[0;0;.63]; u2 = u2./norm(u2);  

% Conjunto dos bambolês:
Bmb = [b1;b2]; Nb = [n1';n2'];
Vb = [v1';v2'];  Ub = [u1';u2'];

%% Plots ==================================================================
p0 = [0,0.5,1]; n0 = [1,0,0];% ponto inicial 
Curva =[];        Curva.Vmax = .1;
[CB,dCB,Curva] = CurvaBmbExp(p0,n0,Bmb,Nb,Vb,Ub,Curva);

%% Experimento ============================================================
% detect rigid body ID from optitrack
rb = OPT.RigidBody;          % read optitrack data
A = getOptData(rb(idA),A(1));   % get ardrone data

% Conectar Joystick
J = JoyControl;

% Conectando no Drone
A.rConnect;

% ArDrone Takeoff
disp('Start take off timing....');
A.rTakeOff;
% pause(2)
disp('End taking off timer....');

XX = []; % Dados armazenados
ps = p0; % ponto inicial

t = tic;
tc = tic;
tp = tic;
rho = 0;

try
while J.pFlag == 0
    if toc(tc) > A.pPar.Ts
        tc = tic;
        tt = toc(t);

        clc
        fprintf('Posição no caminho: %i \n',Curva.Pos)
        fprintf('Percorrendo: %0.2g%% \n',(Curva.Pos/Curva.Size)*100)
        
        %             12        12      1   
        XX = [XX [A.pPos.Xd; A.pPos.X; tt]];

        % Get optitrack data        
        rb = OPT.RigidBody;             % read optitrack
        A = getOptData(rb(idA),A);
        
        Curva.rho = [Curva.rho,rho];
        
        % Controlador
        [A,Curva,rho] = cPathFollowing(A,Curva,Curva.Vmax);
        
        A.pPos.Xr(1:3) = A.pPos.Xd(1:3);
        A.pPos.Xr(6) = A.pPos.Xd(6);
        A.pPos.Xr(7:9) = A.pPos.Xd(7:9);
        
%         if rho>0.2
            A.pSC.Kinematics_control = 0;
%         else
%             A.pSC.Kinematics_control = 1;
%         end
        A = cInverseDynamicController_Compensador_ArDrone(A);
        
        if Curva.Pos==Curva.Size
            A.rLand;
        end
        A = J.mControl(A);
        
        A.rSendControlSignals

        flag=0;
        if J.pFlag == 1
            break 
        end
    end
end
catch ME
    disp(ME)
    A.rLand;
end
A.rLand;
A.rDisconnect;

clc
disp('Calculado!')
disp('Desenhando cena...')

%% Resimulando ============================================================
disp('Pronto para reproduzir.')
disp('Reproduzindo simulação...')
hold on
plot3(XX(13,:),XX(14,:),XX(15,:),'g-','LineWidth',2)
grid on
for tr = 1:5:size(XX,2)
    A.pPos.X = XX([13:24],tr);
    A.mCADplot;
    drawnow
end

%% Salvar dados ===========================================================
dt = datestr(now,30);
dt = dt(end-6:end);
txt = ['uav_up_exp_',dt,'.mat'];
save(txt)

% Erro de caminho
figure(2)
plot(Curva.rho,'b-','LineWidth',1.3)
hold on
plot(1:size(Curva.rho,2),(Curva.Vmax).*ones(1,size(Curva.rho,2)),'g--','LineWidth',1.3)
ylim([0,1.65*Curva.Vmax])
title('Erro de caminho')
xlabel('Pos')
ylabel('Til')


