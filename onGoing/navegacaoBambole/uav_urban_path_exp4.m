%% Simulação do planejamento de trajetória para bambolês (wayposes)
% - Implementar seguimento de caminho
% - Analisar velocidade/curvatura no tempo, se não tiver ok corrigir
% - Distorcer curva para proteger bambolês passados
% A.pPar.Battery
% hardlim = z >= 1.5*0.63

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
close all
% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
A = ArDrone(1);
B1 = ArDrone(2); % Bambolê Livre

% detect rigid body ID from optitrack
idA = getID(OPT,ArDrone,1);     % ID do bambolê no optitrack
idB1 = getID(OPT,ArDrone,2);     % ID do bambolê livre no optitrack

rb = OPT.RigidBody;          % read optitrack data
A = getOptData(rb(idA),A);   % get ardrone data
B1 = getOptData(rb(idB1),B1);   % get ardrone data
% Informações geratrizes:
p0 = [0,1.3,1]; n0 = [-1,0,0];% ponto inicial 
Curva =[];      Curva.Vmax = .1;

figure
axis([-4 3 -4 3 0 3])
view(228.1000,49.1520)
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
grid on

[p0,n0,Bmb,Nb,Vb,Ub,Curva] = CorrigeCurva(A,B1,Curva);
[CB,dCB,Curva] = CurvaBmbExp(p0,n0,Bmb,Nb,Vb,Ub,Curva);
%% Experimento ============================================================
% detect rigid body ID from optitrack
rb = OPT.RigidBody;          % read optitrack data
A = getOptData(rb(idA),A);   % get ardrone data

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

t = tic;
tc = tic;
tp = tic;
rho = 0;

% Curva = Crc; CB = []; dCB=[];

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
        rb = OPT.RigidBody;  % read optitrack
        A = getOptData(rb(idA),A); % Atualiza dados ArDrone
        B1 = getOptData(rb(idB1),B1); % Atualiza dados bambolê livre
        


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

% clc
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

    
%     d1 = norm(A.pPos.X(1:3)-Bmb-dD.*(Nb/norm(Nb)));
%     d2 = norm(A.pPos.X(1:3)-Bmb+dD.*(Nb/norm(Nb)));
%     Dist = [d1,d2];
%     maior = max(Dist);
%     idx = find(Dist==maior);
