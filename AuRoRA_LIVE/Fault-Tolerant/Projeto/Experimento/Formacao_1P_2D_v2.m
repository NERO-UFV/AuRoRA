close all
clear all
clc

try
    fclose(instrfindall);
catch
end
%% ADD TO PATH
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% INICIO DO PROGRAMA
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% CARREGANDO AS CLASSES
% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

A{1} = ArDrone; % Drone em cima do Pioneer3DX
A{2} = ArDrone; % Drone voando
P = Pioneer3DX; % Pioneer3DX
idA{1} = getID(OPT,ArDrone,1);
idA{2} = getID(OPT,ArDrone,2);
idP = getID(OPT,P);            % pioneer ID on optitrack
A{1}.pPar.ti = tic;
A{2}.pPar.ti = tic;
rb = OPT.RigidBody;
if rb(idA{1}).isTracked
    A{1} = getOptData(rb(idA{1}),A{1});
    A{1}.pPos.X(9)
end

if rb(idA{2}).isTracked
    A{2} = getOptData(rb(idA{2}),A{2});
    A{2}.pPos.X
end

% Joystick
J = JoyControl;

A{1}.pPos.X(1:3) = [0 0 0.3225]; % Altura do modelo do pioneer 0.3225

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Variáveis iniciais

L = .8*diag([1 1 1 1 1 1 1 1 1]); % Ganho formação

% Bebop e Ardrone Takeoff
disp('Start Take Off Timming....');
A{1}.pPar.ip = '192.168.1.30';
A{1}.rConnect;
A{1}.pPar.ti = tic;

A{2}.pPar.LocalPortControl = 5558;
A{2}.pPar.LocalPortState = 5552;
A{2}.pPar.ip = '192.168.1.50';
A{2}.rConnect;

% A{1}.rTakeOff;
A{2}.rTakeOff;


pause(5);
A{2}.pPar.ti = tic;
P.pPar.ti = tic;
disp('Taking Off End Time....');

phi = 0;
psi = 0;
theta = 0;
beta = pi/4;
raio(1) = 2;
raio(2) = raio(1)*beta;

X_d2 = (rand-0.5)*5;
if X_d2 >= 0
    X_d2 = X_d2 + raio(1);
else
    X_d2 = X_d2 - raio(1);
end
Y_d2 = (rand-0.5)*5;
if Y_d2 >= 0
    Y_d2 = Y_d2 + raio(1);
else
    Y_d2 = Y_d2 - raio(1);
end
Quad = 0;
switch Quad
    case 1
        %1 Quadrante
        X_d2 = 4;
        Y_d2 = 4;
    case 2
        %2 Quadrante
        X_d2 = -4;
        Y_d2 = 4;
    case 3
        %3 Quadrante
        X_d2 = -4;
        Y_d2 = -4;
    case 4
        %4 Quadrante
        X_d2 = 4;
        Y_d2 = -4;
end
Z_d2 = rand*5 + 2;
Z_d2 = 1.8;
psi = atan2(Y_d2,X_d2) + pi;

X_d2 = 0;
Y_d2 = -1;
% Z_d2 = A{2}.pPos.X(3);

psi = atan2(Y_d2,X_d2) + pi;
% A{2}.pPos.X(1:3) = [X_d2 Y_d2 raio(1)*cos(beta)];
% A{2}.pPos.X(1:3) = [X_d2 Y_d2 0];

%% LEITURA DE POSIÇÃO E VELOCIDADE

% OPTITRACK
rb = OPT.RigidBody;
% if rb(idA{1}).isTracked
%     A{1} = getOptData(rb(idA{1}),A{1});
% %     A{1}.pPos.X
% end

if rb(idA{2}).isTracked
    A{2} = getOptData(rb(idA{2}),A{2});
    A{2}.pPos.X
end

if rb(idP).isTracked
    P = getOptData(rb(idP),P);
%     P.pPos.X
end

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Formação Inicial
Q = [0;              % x
     0;              % y
     0;              % z
     phi;            % phi
     psi;            % psi
     theta;          % tetha
     raio(1);           % p
     raio(1);           % q
     beta];          % beta
    
Qd = Q;

% Posição dos Robôs
X = FT_inversa2(Q);
X_c = FT_inversa(Q);
Xd = X;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Esfera limite
E = 0:pi/32:pi;
I = ones(1,2);

X_E = [raio(1)*cos(psi)*cos(E);
       raio(1)*sin(psi)*cos(E);
       raio(1)*sin(E)];
   
X_C = [raio(2)*cos(psi)*I;
       raio(2)*sin(psi)*I;
       [E(1) E(end)]];

% R rotaciona em relação ao eixo-z
R = [cos(pi/10)  -sin(pi/10)  0;
     sin(pi/10)  cos(pi/10)   0;
     0           0            1];

X_Ea = X_E;
X_E = R*X_E;

X_Ca = X_C;
X_C = R*X_C;
 
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Figura da simulação
figure
% H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b');
hold on
% H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b');
% H(3) = plot3([X(4) X(7)],[X(5) X(8)],[X(6) X(9)],'r');
% A{1}.mCADplot;
% A{2}.mCADplot;
% P.mCADplot(1,'b');
grid on

for j=1:10
for i=1:(size(X_E,2)-1)
%     plot3([X_E(1,i) X_E(1,i+1)],[X_E(2,i) X_E(2,i+1)],[X_E(3,i) X_E(3,i+1)],'k');
%     plot3([X_Ea(1,i) X_Ea(1,i+1)],[X_Ea(2,i) X_Ea(2,i+1)],[X_Ea(3,i) X_Ea(3,i+1)],'k');
%     plot3(X_E(1,i),X_E(2,i),X_E(3,i),'r*');
%     plot3(X_Ea(1,i),X_Ea(2,i),X_Ea(3,i),'r*');
    patch([X_Ea(1,i) X_Ea(1,i+1) X_E(1,i+1) X_E(1,i)],...
        [X_Ea(2,i) X_Ea(2,i+1) X_E(2,i+1) X_E(2,i)],...
        [X_Ea(3,i) X_Ea(3,i+1) X_E(3,i+1) X_E(3,i)],'r',...
        'EdgeColor','k','FaceAlpha',0.5);
end
    X_Ea = X_E;
    X_E = R*X_E;
end

i = 1;
for j = 1:10*2

    patch([X_Ca(1,i) X_Ca(1,i+1) X_C(1,i+1) X_C(1,i)],...
        [X_Ca(2,i) X_Ca(2,i+1) X_C(2,i+1) X_C(2,i)],...
        [X_Ca(3,i) X_Ca(3,i+1) X_C(3,i+1) X_C(3,i)],'b',...
        'EdgeColor','k','FaceAlpha',0.5);

    X_Ca = X_C;
    X_C = R*X_C;
end

AX = raio(1)*2;
axis([-AX AX -AX AX 0 AX])
axis equal
az = psi*180/pi;
el = 45;
view([az el])

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Variáveis iniciais
Tmax = 30;
T_run = 1/30;       % Tempo de amostragem
ta = 1/30;
tp = 0.5;
t1 = tic;
t2 = tic;
% A.pPar.ti = tic;

% pause(1)
t = tic;
tPlot = tic;
Etapa = 1;
T_E = [10 20 5 5 20];
T_e = [5];

A{1}.pPar.ti = tic;
A{2}.pPar.ti = tic;
A{1}.pPar.Ts = 1/30;
A{2}.pPar.Ts = 1/30;

dados = [];
disp('Etapa 1')
%% SIMULAÇÃO
T_Alfa = tic;
while toc(t) < Tmax
    if toc(t1) > ta
        t1 = tic;
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
        % OBJETIVO DA MISSÃO PARA CADA CASO
        switch Etapa
            case 1
                if toc(t) > T_E(1) || (A{2}.pPos.X(1) - P.pPos.X(1))^2 + (A{2}.pPos.X(2) - P.pPos.X(2))^2 <= raio(2)^2
                    X_A = A{2}.pPos.X(1);
                    Y_A = A{2}.pPos.X(2);
                    if (A{2}.pPos.X(1) - P.pPos.X(1))^2 + (A{2}.pPos.X(2) - P.pPos.X(2))^2 + (A{2}.pPos.X(3) - P.pPos.X(3))^2 <= raio(1)^2
                        Etapa = Etapa + 1;
                    else
                        Etapa = Etapa + 1;
                    end
                    disp('Etapa 2')
                    A{1}.rTakeOff;
                    A{1}.pPar.ti = tic;
                    t = tic;
                end
            case 1.1
                if toc(t) > T_e(1) || (A{2}.pPos.X(1) - P.pPos.X(1))^2 + (A{2}.pPos.X(2) - P.pPos.X(2))^2 + (A{2}.pPos.X(3) - P.pPos.X(3))^2 >= raio(1)^2
                    Etapa = Etapa + 0.9;
                    t = tic;
                    disp('Etapa 2')
                end
            case 1.2
                if toc(t) > T_e(1) || (A{2}.pPos.X(1) - P.pPos.X(1))^2 + (A{2}.pPos.X(2) - P.pPos.X(2))^2 + (A{2}.pPos.X(3) - P.pPos.X(3))^2 <= raio(1)^2
                    Etapa = Etapa + 0.8;
                    t = tic;
                    disp('Etapa 2')
                end
            case 2
                if toc(t) > T_E(2)
                    Etapa = Etapa + 1;
                    t = tic;
                    A{1}.pSC.Kinematics_control = 1;
                    A{2}.pSC.Kinematics_control = 1;
                    disp('Etapa 3')
                end
            case 3
                if toc(t) > T_E(3)
                    Etapa = Etapa + 1;
                    t = tic;
                    A{1}.pSC.Kinematics_control = 0;
                    A{2}.pSC.Kinematics_control = 0;
                    disp('Etapa 4')
                end
            case 4
                if toc(t) > T_E(4)
                    Etapa = Etapa + 1;
                    t = tic;
                    Ang = pi-acos(A{1}.pPos.X(3)/raio(1));
                    disp('Etapa 5')
                    A{2}.rLand;
                    A{1}.rLand;
                end
            case 5
                if toc(t) > T_E(5)
                    break
                end
        end
        switch Etapa
            case 1 % Drone 2 voa até o cilindro
                A{2}.pPos.Xd([1:3 6]) = [X_d2 + toc(t)/T_E(1)*(P.pPos.X(1) - X_d2);
                                         Y_d2 + toc(t)/T_E(1)*(P.pPos.X(2) - Y_d2);
                                         Z_d2 + toc(t)/T_E(1)*(P.pPos.X(3) - Z_d2);
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [(P.pPos.X(1) - X_d2)/T_E(1);
                                          (P.pPos.X(2) - Y_d2)/T_E(1);
                                          (P.pPos.X(3) - Z_d2)/T_E(1);
                                          0];
                A{2}.pPos.Xd([1:3 6]) = [X(7);
                                         X(8);
                                         X(9);
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [0;
                                          0;
                                          0;
                                          0];
            case 1.1 % Drone 2 posiciona para iniciar a formação
                A{2}.pPos.Xd([1:3 6]) = [A{2}.pPos.X(1) + toc(t)/T_E(1)*(X_A - A{2}.pPos.X(1));
                                         A{2}.pPos.X(2) + toc(t)/T_E(1)*(Y_A - A{2}.pPos.X(2));
                                         A{2}.pPos.X(3) + toc(t)/T_E(1)*(raio(1)*cos(beta) - A{2}.pPos.X(3));
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [(X_A - A{2}.pPos.X(1))/T_E(1);
                                          (Y_A - A{2}.pPos.X(2))/T_E(1);
                                          (raio(1)*cos(beta) - A{2}.pPos.X(3))/T_E(1);
                                          0];
            case 1.2 % Drone 2 posiciona para iniciar a formação
                A{2}.pPos.Xd([1:3 6]) = [A{2}.pPos.X(1) + toc(t)/T_E(1)*(X_A - A{2}.pPos.X(1));
                                         A{2}.pPos.X(2) + toc(t)/T_E(1)*(Y_A - A{2}.pPos.X(2));
                                         A{2}.pPos.X(3) + toc(t)/T_E(1)*(raio(1)*cos(beta) - A{2}.pPos.X(3));
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [(X_A - A{2}.pPos.X(1))/T_E(1);
                                          (Y_A - A{2}.pPos.X(2))/T_E(1);
                                          (raio(1)*cos(beta) - A{2}.pPos.X(3))/T_E(1);
                                          0];
            case 2 % Drone 1 levanta voo e inicia a formação
                A{1}.pPos.Xd([1:3 6]) = [0;
                                         0;
                                         0.3225 + (raio(1) - 0.3225)*toc(t)/T_E(2);
                                         0];
                A{1}.pPos.Xd([7:9 12]) = [0;
                                          0;
                                          (raio(1) - 0.3225)/T_E(2);
                                          0];
                if toc(t) < T_E(2)/2
                    A{1}.pPos.Xd([1:3 6]) = [0;
                      0;
                      raio(1)-0.5;
                      0];
                else
                    A{1}.pPos.Xd([1:3 6]) = [0;
                      0;
                      raio(1);
                      0];
                end
                A{1}.pPos.Xd([7:9 12]) = [0;
                                          0;
                                          0;
                                          0];
                A{2}.pPos.Xd([1:3 6]) = [X(7);
                                         X(8);
                                         X(9);
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [0;
                                          0;
                                          0;
                                          0];
                                         
            case 3 % Formação movimenta colocando o A{1} 2 em cima do pioneer
                Qd = [0;                % x
                      0;                % y
                      0;                % z
                      beta*toc(t)/T_E(3);% phi
                      psi;              % psi
                      theta;            % theta
                      raio(1);             % p
                      raio(1);             % q
                      beta];            % beta
                 dQd = [0;              % x
                      0;                % y
                      0;                % z
                      beta/T_E(3);       % phi
                      0;                % psi
                      0;                % tetha
                      0;                % p
                      0;                % q
                      0];               % beta
                  Xd = FT_inversa2(Qd);
                  Qtil = Qd - Q;
                  dQ = dQd + L*tanh(Qtil);
                  dX = FT_jacobianoInv2(Q,dQ);
                  dX_c = FT_jacobianoInv(Q,dQ);
                  A{1}.pPar.Xr([7:9]) = dX([4:6]);
                  A{2}.pPar.Xr([7:9]) = dX([7:9]);
%                  dX = Jinv(Q)*dQ
%                  dX no controle
%                  lê X e acha Q
            case 4 % Drone 2 pousa no pioneer e formação é desfeita
                A{1}.pPos.Xd([1:3 6]) = A{1}.pPos.X([1:3 6]);
                A{1}.pPos.Xd([7:9 12]) = [0 0 0 0]';
                A{2}.pPos.Xd([1:3 6]) = [0;
                                         0;
                                         raio(1) + (0.3225 - raio(1))*toc(t)/T_E(4);
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [0;
                                          0;
                                          (0.3225 - raio(1))/T_E(4);
                                          0];
                A{2}.pPos.Xd([1:3 6]) = [0;
                                         0;
                                         X(9);
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [0;
                                          0;
                                          0;
                                          0];
            case 5 % Drone 1 segue voando, agora em cima do pioneer
                A{1}.pPos.Xd([1:3 6]) = [raio(1)*cos(psi+pi)*cos(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                         raio(1)*sin(psi+pi)*cos(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                         raio(1)*sin(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                         0];
                A{1}.pPos.Xd([7:9 12]) = [-((pi/2 - Ang)/T_E(5))*raio(1)*cos(psi+pi)*sin(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                          -((pi/2 - Ang)/T_E(5))*raio(1)*sin(psi+pi)*sin(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                          ((pi/2 - Ang)/T_E(5))*raio(1)*cos(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                          0];
                A{2}.pPos.Xd([1:3 6]) = A{2}.pPos.X([1:3 6]);
                A{2}.pPos.Xd([7:9 12]) = [0 0 0 0]';
        end
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % LEITURA DE POSIÇÃO E VELOCIDADE
        
        % OPTITRACK
        rb = OPT.RigidBody;
%         if rb(idA{1}).isTracked
%             A{1} = getOptData(rb(idA{1}),A{1});
% %             A{1}.pPos.X
%         end
        %
        if rb(idA{2}).isTracked
            A{2} = getOptData(rb(idA{2}),A{2});
%             A{2}.pPos.X
        end
        
        if rb(idP).isTracked
            P = getOptData(rb(idP),P);
            %     P.pPos.X
        end
        
%         P.rGetSensorData;
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % CONTROLE PARA CADA CASO
        switch Etapa
            case 1
                A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
            case 1.1
                A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
            case 1.2
                A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
            case 2
                A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});%% Send control signals
                A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
            case 3
                A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});%% Send control signals
                A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
            case 4
                A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});%% Send control signals
                A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
            case 5
                A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});%% Send control signals
                A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
        end
        
        if Etapa > 1 
            A{1} = J.mControl(A{1});                    % joystick command (priority)
            if Etapa == 2 && toc(t) > 3
                A{1}.rSendControlSignals;
            else
                A{1}.rSendControlSignals;
            end
        end
        A{2} = J.mControl(A{2});                    % joystick command (priority)
        A{2}.rSendControlSignals;
        
        if Etapa == 3
            X([4:6]) = A{1}.pPos.X([1:3]);
            X([7:9]) = A{2}.pPos.X([1:3]);
            Q = FT_direta2(X);
            Q_c = FT_direta(X);
        end
        
        A{1}.pPar.ti = tic;
        A{2}.pPar.ti = tic;
        
        dados(end+1,:) = [A{1}.pPos.Xd' A{1}.pPos.X' A{1}.pSC.Ud' A{1}.pSC.U'...
            A{2}.pPos.Xd' A{2}.pPos.X' A{2}.pSC.Ud' A{2}.pSC.U'...
            P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' Qd' Q' Xd' X'...
            A{1}.pPar.Xr' A{2}.pPar.Xr' toc(T_Alfa)];
            
%           1 -- 12         13 -- 24        25 -- 28        29 -- 32
%           A{1}.pPos.Xd'   A{1}.pPos.X'    A{1}.pSC.Ud'    A{1}.pSC.U'
%
%           33 -- 44        45 -- 56        57 -- 60        61 -- 64
%           A{2}.pPos.Xd'   A{2}.pPos.X'    A{2}.pSC.Ud'    A{2}.pSC.U'
%
%           65 -- 76        77 -- 88        89 -- 90        91 -- 92
%           P.pPos.Xd'      P.pPos.X'       P.pSC.Ud'       P.pSC.U'
% 
%           93 -- 101       102 -- 110      111 -- 119      120 -- 128
%           Qd'             Q'              Xd'             X'
% 
%           129 -- 140      141 -- 152      153
%           A{1}.pPar.Xr'   A{2}.pPar.Xr'   toc(T_Alfa)

        drawnow
    end
%     if toc(tPlot) > tp
%         tPlot = tic;
%         try
%             P.mCADdel;
%             delete(H);
%         end
%         
%         A{1}.mCADplot;
%         A{2}.mCADplot;
%         P.mCADplot(1,'b');
%         if Etapa == 3
%             H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b');
%             H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b');
%             H(3) = plot3([X(4) X(7)],[X(5) X(8)],[X(6) X(9)],'r');
%         end
%         plot3(A{2}.pPos.X(1),A{2}.pPos.X(2),A{2}.pPos.X(3),'.k');
%         plot3(A{2}.pPos.Xd(1),A{2}.pPos.Xd(2),A{2}.pPos.Xd(3),'.r');
%         drawnow
%     end
end

figure
subplot(2,3,1)
plot(dados(:,end),dados(:,12+1))
hold on
plot(dados(:,end),dados(:,128+7))
grid on
title('X')
subplot(2,3,2)
plot(dados(:,end),dados(:,12+2))
hold on
plot(dados(:,end),dados(:,128+8))
grid on
title('Y')
subplot(2,3,3)
plot(dados(:,end),dados(:,12+3))
hold on
plot(dados(:,end),dados(:,128+9))
grid on
title('Z')
subplot(2,3,4)
plot(dados(:,end),dados(:,44+1))
hold on
plot(dados(:,end),dados(:,140+7))
grid on
title('X')
subplot(2,3,5)
plot(dados(:,end),dados(:,44+2))
hold on
plot(dados(:,end),dados(:,140+8))
grid on
title('Y')
subplot(2,3,6)
plot(dados(:,end),dados(:,44+3))
hold on
plot(dados(:,end),dados(:,140+9))
grid on
title('Z')

figure
subplot(3,3,1)
plot(dados(:,end),(dados(:,0+1)-dados(:,12+1)))
grid on
title('X')
subplot(3,3,2)
plot(dados(:,end),(dados(:,0+2)-dados(:,12+2)))
grid on
title('Y')
subplot(3,3,3)
plot(dados(:,end),(dados(:,0+3)-dados(:,12+3)))
grid on
title('Z')
subplot(3,3,4)
plot(dados(:,end),(dados(:,32+1)-dados(:,44+1)))
grid on
title('X')
subplot(3,3,5)
plot(dados(:,end),(dados(:,32+2)-dados(:,44+2)))
grid on
title('Y')
subplot(3,3,6)
plot(dados(:,end),(dados(:,32+3)-dados(:,44+3)))
grid on
title('Z')
subplot(3,3,7)
plot(dados(:,end),(dados(:,64+1)-dados(:,76+1)))
grid on
title('X')
subplot(3,3,8)
plot(dados(:,end),(dados(:,64+2)-dados(:,76+2)))
grid on
title('Y')
subplot(3,3,9)
plot(dados(:,end),(dados(:,64+3)-dados(:,76+3)))
grid on
title('Z')

figure
subplot(3,3,1)
plot(dados(:,end),dados(:,0+1))
hold on
plot(dados(:,end),dados(:,12+1))
grid on
title('X')
subplot(3,3,2)
plot(dados(:,end),dados(:,0+2))
hold on
plot(dados(:,end),dados(:,12+2))
grid on
title('Y')
subplot(3,3,3)
plot(dados(:,end),dados(:,0+3))
hold on
plot(dados(:,end),dados(:,12+3))
grid on
title('Z')
subplot(3,3,4)
plot(dados(:,end),dados(:,32+1))
hold on
plot(dados(:,end),dados(:,44+1))
grid on
title('X')
subplot(3,3,5)
plot(dados(:,end),dados(:,32+2))
hold on
plot(dados(:,end),dados(:,44+2))
grid on
title('Y')
subplot(3,3,6)
plot(dados(:,end),dados(:,32+3))
hold on
plot(dados(:,end),dados(:,44+3))
grid on
title('Z')
subplot(3,3,7)
plot(dados(:,end),dados(:,64+1))
hold on
plot(dados(:,end),dados(:,76+1))
grid on
title('X')
subplot(3,3,8)
plot(dados(:,end),dados(:,64+2))
hold on
plot(dados(:,end),dados(:,76+2))
grid on
title('Y')
subplot(3,3,9)
plot(dados(:,end),dados(:,64+3))
hold on
plot(dados(:,end),dados(:,76+3))
grid on
title('Z')

figure
subplot(3,3,1)
plot(dados(:,end),dados(:,110+1))
hold on
plot(dados(:,end),dados(:,119+1))
grid on
title('X')
subplot(3,3,2)
plot(dados(:,end),dados(:,110+2))
hold on
plot(dados(:,end),dados(:,119+2))
grid on
title('Y')
subplot(3,3,3)
plot(dados(:,end),dados(:,110+3))
hold on
plot(dados(:,end),dados(:,119+3))
grid on
title('Z')
subplot(3,3,4)
plot(dados(:,end),dados(:,110+4))
hold on
plot(dados(:,end),dados(:,119+4))
grid on
title('X')
subplot(3,3,5)
plot(dados(:,end),dados(:,110+5))
hold on
plot(dados(:,end),dados(:,119+5))
grid on
title('Y')
subplot(3,3,6)
plot(dados(:,end),dados(:,110+6))
hold on
plot(dados(:,end),dados(:,119+6))
grid on
title('Z')
subplot(3,3,7)
plot(dados(:,end),dados(:,110+7))
hold on
plot(dados(:,end),dados(:,119+7))
grid on
title('X')
subplot(3,3,8)
plot(dados(:,end),dados(:,110+8))
hold on
plot(dados(:,end),dados(:,119+8))
grid on
title('Y')
subplot(3,3,9)
plot(dados(:,end),dados(:,110+9))
hold on
plot(dados(:,end),dados(:,119+9))
grid on
title('Z')