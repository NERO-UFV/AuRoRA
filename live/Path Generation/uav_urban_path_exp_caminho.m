%% Simula��o do planejamento de trajet�ria para bambol�s (wayposes)
% - Implementar seguimento de caminho
% - Analisar velocidade/curvatura no tempo, se n�o tiver ok corrigir
% - Distorcer curva para proteger bambol�s passados

% Boas pr�ticas
close all
clear
clc

% CONTROLAR TEMPO TOTAL DE SIMULA��O: (min = 100s)
T = 30;
% 150;

try
    fclose(instrfindall)
    % Rotina para buscar pasta raiz
    PastaAtual = pwd;
    PastaRaiz = 'AuRoRA 2018';
    cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
    addpath(genpath(pwd))
    
    disp('Diret�rio raiz e caminho bem definidos!')
    
catch
    disp('Diret�rio raiz errado!')
end

% =========================================================================
% Defini��o da cena:
%Diametro do A real (63 cm)
dD = .8; 

% Bambol�s:
% b1 =[1;1;1];     n1 = [-1;-0.5;1];
% v1 = [1;0;1];    u1 = [0;1;0.5];
% 
% 
% b2 = [3;-1;1];     n2 = [-1;-1;1].*-0.25;
% v2 = [0.5;0;0.5];   u2 = [0;0.5;0.5];
% 
% b3 = [-2;-2;1.5];   n3 = [1;0;0];
% v3 = [0;-1;0];      u3 = [0;0;1];


% Conjunto dos bambol�s:
% Bmb = [b1,b2,b3]; Nb = [n1,n2,n3];
% Vb = [v1,v2,v3];  Ub = [u1,u2,u3];

% Sem optitrack
% b4 = [0;0;1.35]; n4 = [0;1;0];
% v4 = [1;0;0];   u4 = [0;0;1];

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
A = ArDrone(1);
B1 = ArDrone(2);
B2 = ArDrone(3);


% detect rigid body ID from optitrack
idA = getID(OPT,ArDrone,1);     % ID do bambol� no optitrack
idB1 = getID(OPT,ArDrone,2);     % ID do bambol� 1 no optitrack
idB2 = getID(OPT,ArDrone,3);     % ID do bambol� 2 no optitrack


rb = OPT.RigidBody;          % read optitrack data

B1 = getOptData(rb(idB1),B1);   % get hulla-hoop 1 data
B2 = getOptData(rb(idB2),B2);   % get hulla-hoop 1 data

% Orienta��o:
% Rx = [1 0 0; 0 cos(drone.pPos.X(4)) -sin(drone.pPos.X(4));...
%               0 sin(drone.pPos.X(4)) cos(drone.pPos.X(4))];
% Ry = [cos(drone.pPos.X(5)) 0 sin(drone.pPos.X(5)); 0 1 0;...
%               -sin(drone.pPos.X(5)) 0 cos(drone.pPos.X(5))];
Rz1 = [cos(B1.pPos.X(6)) -sin(B1.pPos.X(6)) 0;...
       sin(B1.pPos.X(6)) cos(B1.pPos.X(6)) 0; 0 0 1];
   
Rn = [cos((90)*(pi/180)) -sin((90)*(pi/180)) 0;...
      sin((90)*(pi/180)) cos((90)*(pi/180)) 0; 0 0 1];

Rz2 = [cos(B2.pPos.X(6)) -sin(B2.pPos.X(6)) 0;...
       sin(B2.pPos.X(6)) cos(B2.pPos.X(6)) 0; 0 0 1];

b1 = B1.pPos.X(1:3); 
n1 = Rz1*[1;0;0]; n1 = Rn*n1;
v1 = B1.pPos.X(1:3)+[0;0.63;0];  
u1 = B1.pPos.X(1:3)+[0;0;.63];  

b2 = B2.pPos.X(1:3); 
n2 = Rz2*[1;0;0]; n2 = -Rn*n2;
v2 = B2.pPos.X(1:3)+[0;0.63;0];  
u2 = B2.pPos.X(1:3)+[0;0;.63];  

% Conjunto dos bambol�s:
Bmb = [b1,b2]; Nb = [n1,n2];
Vb = [v1,v2];  Ub = [u1,u2];

% Bmb = [b1]; Nb = [n1];
% Vb = [v1];  Ub = [u1];

%% Plots ==================================================================
ps = [0;1.5;1]; % ponto inicial
CB = [];

figure(1)
axis([-3 4 -4 3 0 3])
view(100,42)
grid on
xlabel('X [m]')
ylabel('Y [m]')

N = 250;

for i = 1:size(Nb,2)
    if i>1    
        ps = [pfd-2*dD.*(n/norm(n)),pfd-3*dD.*(n/norm(n))]; 
        ControlPoints = [pfd,ps(:,1)];
        
        time = linspace(0,1,N);
        
        CB = [CB, DBezierCurve(ControlPoints,time)];
    end
    
    pf = Bmb(:,i);
    n  = Nb(:,i);
    V = Vb(:,i)/norm(Vb(:,i));
    U = Ub(:,i)/norm(Ub(:,i));
     
    % Deslocamento do ponto final:
    pfd = pf +dD.*(n/norm(n)); % ponto final deslocado por seguran�a   
    dp = pfd-ps(:,1); % ponto inicial ao ponto deslocado

    % Defini��o Retas:
    t = -1.5:0.1:1.5;
    L = t.*n; % passa pelo bambol�
    R = t.*dp; % do A ao bambol�
    z = t.*cross(R,L)+pfd; % normal ao plano

    % Ponto de controle:
    idx = find(abs(n)==max(abs(n)));
    pc = pfd + n.*dD;

    % Desenho dos pontos e retas ==========================================
    % Ponto inicial:                  (PRETO)
    hold on
    plot3(ps(1),ps(2),ps(3),'ko')
    hold on
    plot3(ps(1),ps(2),ps(3),'k*')

    % Ponto final:                  (VERMELHO)
    hold on
    plot3(pf(1),pf(2),pf(3),'ro')
    hold on
    plot3(pf(1),pf(2),pf(3),'r*')


    % Ponto final deslocado:        (VERMELHO)
    hold on
    plot3(pfd(1),pfd(2),pfd(3),'ro')
    hold on
    plot3(pfd(1),pfd(2),pfd(3),'r*')

    % Ponto de controle:              (AZUL)
    hold on
    plot3(pc(1),pc(2),pc(3),'b*')
    hold on
    plot3(pc(1),pc(2),pc(3),'bo')

    % Normal ao bambol�:
    hold on
    plot3(L(1,:)+pfd(1),L(2,:)+pfd(2),L(3,:)+pfd(3),'b-.','LineWidth',1)

    % Do A ao bambol�:
    hold on
    plot3([pfd(1) ps(1)],[pfd(2) ps(2)],[pfd(3) ps(3)],'r:.','LineWidth',1.2)

    % Bambol� 1: 
    u = 0:0.01:2*pi;
    XB = pf(1) +0.63.*cos(u).*V(1) +0.63*sin(u).*U(1);
    YB = pf(2) +0.63.*cos(u).*V(2) +0.63*sin(u).*U(2);
    ZB = pf(3) +0.63.*cos(u).*V(3) +0.63*sin(u).*U(3);
    hold on
    plot3(XB,YB,ZB,'LineWidth',2)
    grid on

    % Calcular e desenhar curva de B�zier:
    ControlPoints = [ps,pc,pfd];
    time = linspace(0,1,N);
    CB = [CB, DBezierCurve(ControlPoints,time)];

    % Pontos de Controle:
    hold on
    plot3(ControlPoints(1,:),ControlPoints(2,:),...
          ControlPoints(3,:),'r:.','LineWidth',1.2)
      
    if i==size(Nb,2)
        ControlPoints = [pfd,pfd-2*dD.*n];
        time = linspace(0,1,N);
        CB = [CB, DBezierCurve(ControlPoints,time)];
    end
end

hold on
plot3(CB(1,:),CB(2,:),CB(3,:),'b--','LineWidth',2.5)

%% Experimento ==============================================================
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

XX = [];
flag1=1;
tmax = T; % Tempo Simula��o em segundos
X = zeros(1,19); % Dados correntes da simula��o
ControlPoints = [];
ps = [0;1.5;1]; % ponto inicial

k = size(CB,2);
dCB = [[0;0;0],diff(CB,1,2)];
Curva.rho = 0;
Curva.psi = atan2(CB(2,:),CB(1,:));
Curva.dpsi = [0,diff(Curva.psi,1,2)];
Curva.X = CB;
Curva.dX = [[0;0;0],diff(CB,1,2)];
Curva.Pos = 1;
Curva.Size = k;
Path.dXr = zeros(3,k);
Curva.Vmax = 0.07; % Curva.Vmax = 1/10; % => 120 seg

t = tic;
tc = tic;
tp = tic;
try
while J.pFlag == 0
    if toc(tc) > A.pPar.Ts
        tc = tic;
        tt = toc(t);
        TT = (tt/tmax);
        itT = floor(k*TT)+1;     
        
%         clc
        fprintf('Percorrendo: %0.2g%% \n',(Curva.Pos/Curva.Size)*100);
        
        %             12        12      1   
        XX = [XX [A.pPos.Xd; A.pPos.X; tt]];

        % Get optitrack data        
        rb = OPT.RigidBody;             % read optitrack
        A = getOptData(rb(idA),A);

        % Controlador
        [A,Curva] = cPathFollowing(A,Curva);
        A.pSC.Kinematics_control = 1;
        Curva.Pos
        A.pPos.Xr(1:3) = A.pPos.Xd(1:3);
        A.pPos.Xr(6) = A.pPos.Xd(6);
        A.pPos.Xr(7:9) = A.pPos.Xd(7:9);
        
%         if norm(A.pPos.X(1:3)-Curva.X(1:3,Curva.Pos))>0.2
            A.pSC.Kinematics_control = 0;
            A = cInverseDynamicController_Compensador_ArDrone(A);
%         else
%             A.pSC.Kinematics_control = 1;
%             A = cInverseDynamicController_Compensador_ArDrone(A);
%         end
        
%         if Curva.Pos==Curva.Size
%             A.rLand;
%         end
        A = J.mControl(A);
        
        
        A.rSendControlSignals

%         if toc(tp) > 0.3
%             if ~isempty(XX)
%                 plot3(XX(1,:),XX(2,:),XX(3,:),'r-');
%             end
%             tp = tic;
%             A.mCADplot;
%             drawnow
%         end

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
disp('Reproduzindo simula��o...')
hold on
plot3(XX(13,:),XX(14,:),XX(15,:),'g-','LineWidth',2)
grid on
for tr = 1:5:size(XX,2)
    A.pPos.X = XX([13:24],tr);
    A.mCADplot;
    drawnow
end
% =========================================================================
% tmax = 540;
% tmax = 445;
% tmax = 139;

%% Salvar dados ===========================================================
dt = datestr(now,30);
dt = dt(end-6:end);
txt = ['uav_up_exp_',dt,'.mat'];
save(txt)

%         A.pPos.Xd(1) = CB(1,itT);     
%         A.pPos.Xd(2) = CB(2,itT);
%         A.pPos.Xd(3) = CB(3,itT);
%         A.pPos.Xd(6) = Curva.psi(itT);
%         A.pPos.Xd(7) = dCB(1,itT);  
%         A.pPos.Xd(8) = dCB(2,itT);
%         A.pPos.Xd(9) = dCB(3,itT);
%         A.pPos.Xd(6) = Curva.dpsi(itT);

%         % Integra��o Num�rica =============================================
%         % ArDrone
%         Ar = [A.pPar.k1  A.pPar.k1 -A.pPar.k1  -A.pPar.k1;
%             -A.pPar.k1  A.pPar.k1  A.pPar.k1  -A.pPar.k1;
%             A.pPar.k2 -A.pPar.k2  A.pPar.k2  -A.pPar.k2];
%         
%         % Rotational inertia matrix
%         Mr = [A.pPar.Ixx, ...
%             A.pPar.Ixy*cos(A.pPos.X(4)) - A.pPar.Ixz*sin(A.pPos.X(4)), ...
%             -A.pPar.Ixx*sin(A.pPos.X(5)) + A.pPar.Ixy*sin(A.pPos.X(4))*cos(A.pPos.X(5)) + A.pPar.Ixz*cos(A.pPos.X(4))*cos(A.pPos.X(5));
% 
%             A.pPar.Ixy*cos(A.pPos.X(4)) - A.pPar.Ixz*sin(A.pPos.X(4)), ...
%             A.pPar.Iyy*cos(A.pPos.X(4))^2 + A.pPar.Izz*sin(A.pPos.X(4))^2 - 2*A.pPar.Iyz*sin(A.pPos.X(4))*cos(A.pPos.X(4)),...
%             A.pPar.Iyy*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5)) - A.pPar.Izz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5)) - A.pPar.Ixy*cos(A.pPos.X(4))*sin(A.pPos.X(5)) + A.pPar.Ixz*sin(A.pPos.X(4))*sin(A.pPos.X(5)) + A.pPar.Iyz*cos(A.pPos.X(4))^2*cos(A.pPos.X(5)) - A.pPar.Iyz*sin(A.pPos.X(4))^2*cos(A.pPos.X(5));
% 
%             -A.pPar.Ixx*sin(A.pPos.X(5)) + A.pPar.Ixy*sin(A.pPos.X(4))*cos(A.pPos.X(5)) + A.pPar.Ixz*cos(A.pPos.X(4))*cos(A.pPos.X(5)), ...
%             A.pPar.Iyy*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5)) - A.pPar.Izz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5)) - A.pPar.Ixy*cos(A.pPos.X(4))*sin(A.pPos.X(5)) + A.pPar.Ixz*sin(A.pPos.X(4))*sin(A.pPos.X(5)) + A.pPar.Iyz*cos(A.pPos.X(4))^2*cos(A.pPos.X(5)) - A.pPar.Iyz*sin(A.pPos.X(4))^2*cos(A.pPos.X(5)),...
%             A.pPar.Ixx*sin(A.pPos.X(5))^2 + A.pPar.Iyy*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))^2 + A.pPar.Izz*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))^2 - 2*A.pPar.Ixy*sin(A.pPos.X(4))*sin(A.pPos.X(5))*cos(A.pPos.X(5)) - 2*A.pPar.Ixz*cos(A.pPos.X(4))*sin(A.pPos.X(5))*cos(A.pPos.X(5)) + 2*A.pPar.Iyz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5))^2
%             ];
%         
%         % Rotational Coriolis matrix
%         Cr = [ 0, ...
%             A.pPos.X(11)*(A.pPar.Iyy*sin(A.pPos.X(4))*cos(A.pPos.X(5)) - A.pPar.Izz*sin(A.pPos.X(4))*cos(A.pPos.X(4)) + A.pPar.Iyz*cos(A.pPos.X(4))^2 - A.pPar.Iyz*sin(A.pPos.X(4))^2) + A.pPos.X(12)*(-A.pPar.Ixx*cos(A.pPos.X(5))/2 - A.pPar.Iyy*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 + A.pPar.Iyy*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 + A.pPar.Izz*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 - A.pPar.Izz*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 - A.pPar.Ixy*sin(A.pPos.X(4))*sin(A.pPos.X(5)) - A.pPar.Ixz*cos(A.pPos.X(4))*sin(A.pPos.X(5)) + 2*A.pPar.Iyz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5))),...
%             A.pPos.X(11)*(-A.pPar.Ixx*cos(A.pPos.X(5))/2 - A.pPar.Iyy*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 + A.pPar.Iyy*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 + A.pPar.Izz*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 - A.pPar.Izz*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 - A.pPar.Ixy*sin(A.pPos.X(4))*sin(A.pPos.X(5)) - A.pPar.Ixz*cos(A.pPos.X(4))*sin(A.pPos.X(5)) + 2*A.pPar.Iyz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5))) + A.pPos.X(12)*(-A.pPar.Iyy*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5))^2 + A.pPar.Izz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5))^2 + A.pPar.Ixy*cos(A.pPos.X(4))*sin(A.pPos.X(5))*cos(A.pPos.X(5)) - A.pPar.Ixz*sin(A.pPos.X(4))*sin(A.pPos.X(5))*cos(A.pPos.X(5)) - A.pPar.Iyz*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))^2 + A.pPar.Iyz*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))^2);
% 
%             A.pPos.X(10)*(-A.pPar.Ixy*sin(A.pPos.X(4)) - A.pPar.Ixz*cos(A.pPos.X(4))) + A.pPos.X(11)*(-A.pPar.Iyy*sin(A.pPos.X(4))*cos(A.pPos.X(4)) + A.pPar.Izz*sin(A.pPos.X(4))*cos(A.pPos.X(4)) - A.pPar.Iyz*cos(A.pPos.X(4))^2 + A.pPar.Iyz*sin(A.pPos.X(4))^2) + A.pPos.X(12)*(A.pPar.Ixx*cos(A.pPos.X(5))/2 + A.pPar.Iyy*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 - A.pPar.Iyy*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 - A.pPar.Izz*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 + A.pPar.Izz*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 + A.pPar.Ixy*sin(A.pPos.X(4))*sin(A.pPos.X(5)) + A.pPar.Ixz*cos(A.pPos.X(4))*sin(A.pPos.X(5)) - 2*A.pPar.Iyz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5))),...
%             A.pPos.X(10)*(-A.pPar.Iyy*sin(A.pPos.X(4))*cos(A.pPos.X(4)) + A.pPar.Izz*sin(A.pPos.X(4))*cos(A.pPos.X(4)) - A.pPar.Iyz*cos(A.pPos.X(4))^2 + A.pPar.Iyz*sin(A.pPos.X(4))^2),...
%             A.pPos.X(10)*(A.pPar.Ixx*cos(A.pPos.X(5))/2 + A.pPar.Iyy*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 - A.pPar.Iyy*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 - A.pPar.Izz*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 + A.pPar.Izz*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 + A.pPar.Ixy*sin(A.pPos.X(4))*sin(A.pPos.X(5)) + A.pPar.Ixz*cos(A.pPos.X(4))*sin(A.pPos.X(5)) - 2*A.pPar.Iyz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5))) + A.pPos.X(12)*(-A.pPar.Ixx*sin(A.pPos.X(5))*cos(A.pPos.X(5)) + A.pPar.Iyy*sin(A.pPos.X(4))^2*sin(A.pPos.X(5))*cos(A.pPos.X(5)) + A.pPar.Izz*cos(A.pPos.X(4))^2*sin(A.pPos.X(5))*cos(A.pPos.X(5)) + A.pPar.Ixy*sin(A.pPos.X(4))*cos(A.pPos.X(5))^2 - A.pPar.Ixy*sin(A.pPos.X(4))*sin(A.pPos.X(5))^2 + A.pPar.Ixz*cos(A.pPos.X(4))*cos(A.pPos.X(5))^2 - A.pPar.Ixz*cos(A.pPos.X(4))*sin(A.pPos.X(5))^2 + 2*A.pPar.Iyz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*sin(A.pPos.X(5))*cos(A.pPos.X(5)));
% 
%             A.pPos.X(10)*(A.pPar.Ixy*cos(A.pPos.X(4))*cos(A.pPos.X(5)) - A.pPar.Ixz*sin(A.pPos.X(4))*cos(A.pPos.X(5))) + A.pPos.X(11)*(-A.pPar.Ixx*cos(A.pPos.X(5))/2 + A.pPar.Iyy*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 - A.pPar.Iyy*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 - A.pPar.Izz*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 + A.pPar.Izz*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 - 2*A.pPar.Iyz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5))) + A.pPos.X(12)*(A.pPar.Iyy*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5))^2 - A.pPar.Izz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5))^2 - A.pPar.Ixy*cos(A.pPos.X(4))*sin(A.pPos.X(5))*cos(A.pPos.X(5)) + A.pPar.Ixz*sin(A.pPos.X(4))*sin(A.pPos.X(5))*cos(A.pPos.X(5)) + A.pPar.Iyz*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))^2 - A.pPar.Iyz*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))^2),...
%             A.pPos.X(10)*(-A.pPar.Ixx*cos(A.pPos.X(5))/2 + A.pPar.Iyy*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 - A.pPar.Iyy*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 - A.pPar.Izz*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 + A.pPar.Izz*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))/2 - 2*A.pPar.Iyz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5))) + A.pPos.X(11)*(-A.pPar.Iyy*sin(A.pPos.X(4))*cos(A.pPos.X(4))*sin(A.pPos.X(5)) + A.pPar.Izz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*sin(A.pPos.X(5)) - A.pPar.Ixy*cos(A.pPos.X(4))*cos(A.pPos.X(5)) + A.pPar.Ixz*sin(A.pPos.X(4))*cos(A.pPos.X(5)) + A.pPar.Iyz*sin(A.pPos.X(4))^2*sin(A.pPos.X(5)) - A.pPar.Iyz*cos(A.pPos.X(4))^2*sin(A.pPos.X(5))) + A.pPos.X(12)*(A.pPar.Ixx*sin(A.pPos.X(5))*cos(A.pPos.X(5)) - A.pPar.Iyy*sin(A.pPos.X(4))^2*sin(A.pPos.X(5))*cos(A.pPos.X(5)) - A.pPar.Izz*cos(A.pPos.X(4))^2*sin(A.pPos.X(5))*cos(A.pPos.X(5)) - A.pPar.Ixy*sin(A.pPos.X(4))*cos(A.pPos.X(5))^2 + A.pPar.Ixy*sin(A.pPos.X(4))*sin(A.pPos.X(5))^2 - A.pPar.Ixz*cos(A.pPos.X(4))*cos(A.pPos.X(5))^2 + A.pPar.Ixz*cos(A.pPos.X(4))*sin(A.pPos.X(5))^2 - 2*A.pPar.Iyz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*sin(A.pPos.X(5))*cos(A.pPos.X(5))),...
%             A.pPos.X(10)*(A.pPar.Iyy*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5))^2 - A.pPar.Izz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*cos(A.pPos.X(5))^2 - A.pPar.Ixy*cos(A.pPos.X(4))*sin(A.pPos.X(5))*cos(A.pPos.X(5)) + A.pPar.Ixz*sin(A.pPos.X(4))*sin(A.pPos.X(5))*cos(A.pPos.X(5)) + A.pPar.Iyz*cos(A.pPos.X(4))^2*cos(A.pPos.X(5))^2 - A.pPar.Iyz*sin(A.pPos.X(4))^2*cos(A.pPos.X(5))^2) + A.pPos.X(11)*(A.pPar.Ixx*sin(A.pPos.X(5))*cos(A.pPos.X(5)) - A.pPar.Iyy*sin(A.pPos.X(4))^2*sin(A.pPos.X(5))*cos(A.pPos.X(5)) - A.pPar.Izz*cos(A.pPos.X(4))^2*sin(A.pPos.X(5))*cos(A.pPos.X(5)) - A.pPar.Ixy*sin(A.pPos.X(4))*cos(A.pPos.X(5))^2 + A.pPar.Ixy*sin(A.pPos.X(4))*sin(A.pPos.X(5))^2 - A.pPar.Ixz*cos(A.pPos.X(4))*cos(A.pPos.X(5))^2 + A.pPar.Ixz*cos(A.pPos.X(4))*sin(A.pPos.X(5))^2 - 2*A.pPar.Iyz*sin(A.pPos.X(4))*cos(A.pPos.X(4))*sin(A.pPos.X(5))*cos(A.pPos.X(5)))
%             ];
%         
%         % 3: W -> F
%         % Deslocando valores passados
%         A.pPar.F  = A.pPar.Cf*A.pPar.W.^2;
%         
%         % Aerodynamic thrust 
%         Tc = Ar*A.pPar.F - A.pPar.Q;
%         
%         % Numerical integration of rotational movement
%         A.pPos.X(10:12) = Mr\(Tc - Cr*A.pPos.X(10:12))*A.pPar.Ts...
%                                                     + A.pPos.X(10:12);
% 
%         % ArA pose - Numerical integration
%         for ii = 1:6
%             A.pPos.X(ii) = A.pPos.X(ii+6)*A.pPar.Ts...
%                                 + A.pPos.X(ii);
%             if ii > 3
%                 if A.pPos.X(ii) > pi
%                     A.pPos.X(ii) = -2*pi + A.pPos.X(ii);
%                 end
%                 if A.pPos.X(ii) < -pi
%                     A.pPos.X(ii) = 2*pi + A.pPos.X(ii);
%                 end
%             end
%         end
