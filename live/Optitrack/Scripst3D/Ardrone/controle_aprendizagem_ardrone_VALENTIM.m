%% Testar modelo dinâmico do ArDrone em controle de posição
% Inicialização
close all; clear all; clc;

try
    fclose(instrfindall);
catch
end

% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


%% Initialize classes
% Create OptiTrack object and initialize
% % % OPT = OptiTrack;
% % % OPT.Initialize;
% % % 
% % % % Connect Joystick
% % % J = JoyControl;

% Robot initialization
A = ArDrone(30);
% A.rConnect;
% pause(5);
% 
% A.rTakeOff;
% pause(5);

disp('Fim da estabilização!');

%% Robot initial pose
% % idA = getID(OPT,A);           % drone ID on optitrack
% % rb  = OPT.RigidBody;          % read optitrack data again
% % A   = getOptData(rb(idA),A);  % populate robot pose variables

%% Parâmetros iniciais

% Ganhos Dinâmicos
Ku = diag([4.72 6.23 2.65 2.38]);

Kv = diag([0.28 0.53 2.58 1.52]);

Ks = diag([1.00 1.00 1.00 1.00]);

Ky = diag([3.10 3.10 3.10 3.10]);

Kd = diag([1.00 1.00 1.00 1.00]);

GAMA = diag([1  1  1  1  1  1  1  1]);
THETA = [1;  1;  1;  1;  1;  1;  1;  1]; %Funciona
% % % % THETA = [0;  0;  0;  0;  0;  0;  0;  0];
THETA_est = [0;  0;  0;  0;  0;  0;  0;  0];

dXr_max = diag([1.5 0.9375 0.2 0.2094]);

% UAV 1
dX_max = diag([1.60 1.10 0.35 0.25]);

Kc = dX_max - dXr_max;
Kc = 5*Kc;

Uc_ant = [0; 0; 0; 0];
H = [];

Xb = [0; 0; 0; 0];
dXb = [0; 0; 0; 0];
ddXb = [0; 0; 0; 0];


%% Window definition
figure(1)
axis([-4 4 -4 4 -0 4]);
xlabel('Eixo X [m]','FontSize',12,'FontWeight','bold');
ylabel('Eixo Y [m]','FontSize',12,'FontWeight','bold');
zlabel('Eixo Z [m]','FontSize',12,'FontWeight','bold');

A.mCADplot;
hold on;
grid on;
fig(1) = plot3(0,0,0,'r--','LineWidth',2);
fig(2) = plot3(0,0,0,'b','LineWidth',2);
legend([fig(1),fig(2)],'X_{d}','X','Location','northwest');
drawnow;
pause(5);

%% Definições de tempo

YN_plot = 1; % Plotar? Yes or No

T_plot = 2;        % Período de plotagem
T_amos = 0.003;      % Período de amostragem
A.pPar.Ts = 0.003;

T_max = 60;     % Tempo total da simulação
T_aprend = 10;  % Tempo para começar a aprender o modelo

t_amos = tic;
t_plot = tic;
t = tic;

while true
    
    if toc(t_amos) > T_amos
        t_amos = tic;
        
        % Referência de Trajetória
        
        p = 1.75;   % Raio
        w = 1.00;   % Frequência angular
        
        A.pPos.Xd([1 2 3 6]) = [      p*sin(w*toc(t))+0.1                    ;
                                      1.25*p*cos(0.5*w*toc(t))-0.2           ;
                                      1+0.5*sin(w*toc(t))                    ;
                                      pi/6*sin(w*toc(t))                    ];
        
        A.pPos.dXd([1 2 3 6]) = [     w*p*cos(w*toc(t))                      ;
                                      -0.5*w*1.25*p*sin(0.5*w*toc(t))        ;
                                      w*0.5*cos(w*toc(t))                    ;
                                      w*pi/6*cos(w*toc(t))                  ];
                                  
                                          F = [ cos(Xb(4))   -sin(Xb(4))   0    0;
            sin(Xb(4))      cos(Xb(4))   0    0;
            0                 0        1    0;
            0                 0        0    1];
        
% %         F = [   cos(A.pPos.X(6))      -sin(A.pPos.X(6))         0       0      ;
% %                 sin(A.pPos.X(6))      cos(A.pPos.X(6))          0       0      ;
% %                         0                   0                   1       0      ;
% %                         0                   0                   0       1     ];
        
%         rb = OPT.RigidBody;
%         A = getOptData(rb(idA),A);
        A.rGetSensorData;
               
        % Modelo Dinâmico
        ddX = F*Ku*A.pSC.Ud - Kv*A.pPos.X([7 8 9 12]); % Global
        A.pPos.X([7 8 9 12]) = A.pPos.X([7 8 9 12]) + ddX*T_amos;
        A.pPos.X([1 2 3 6]) = A.pPos.X([1 2 3 6]) + A.pPos.X([7 8 9 12])*T_amos;
        
        ddXb = Ku*A.pSC.Ud - Kv*dXb; % Drone
        dXb = dXb + ddXb*T_amos;
        Xb = Xb + dXb*T_amos;
        
        A.pPos.Xtil = A.pPos.Xd - A.pPos.X;
        
        % Controlador Cinemático
        Uc = F\(A.pPos.dXd([1 2 3 6]) + Kc*tanh(Ky*A.pPos.Xtil([1 2 3 6])));
        dUc = (Uc - Uc_ant)/T_amos;
        Uc_ant = Uc;
        
        % Controlador Adaptativo
        g11 = dUc(1) + Kd(1)*(Uc(1)-dXb(1));
        g15 = dXb(1);
        g22 = dUc(2) + Kd(2)*(Uc(2)-dXb(2));
        g26 = dXb(2);
        g33 = dUc(3) + Kd(3)*(Uc(3)-dXb(3));
        g37 = dXb(3);
        g44 = dUc(4) + Kd(4)*(Uc(4)-dXb(4));
        g48 = dXb(4);
        
        G = [ g11   0   0   0   g15   0   0   0 ;
            0   g22   0   0   0   g26   0   0  ;
            0   0   g33   0   0   0   g37   0  ;
            0   0   0   g44   0   0   0   g48 ];
        
        D = diag([THETA(1) THETA(2) THETA(3) THETA(4)]);
        SIGMA = dUc + Kd*(Uc - dXb);
        NETA = diag([dXb(1) dXb(2) dXb(3) dXb(4)])*...
            [THETA(5); THETA(6); THETA(7); THETA(8)];
        
        if toc(t) < T_aprend
            dTHETA_est = 0;
        else
            dTHETA_est = GAMA\G'*(Uc - dXb);
        end
        
        THETA_est = THETA_est + dTHETA_est*T_amos;
        THETA_error = THETA_est - THETA;
        
        A.pSC.Ud = D*SIGMA + NETA + G*THETA_error;
        
        
% % % %         % Controlador Cinemático
% % % %         Uc = F\(A.pPos.dXd([1 2 3 6]) + Kc*tanh(Ky*A.pPos.Xtil([1 2 3 6])));
% % % %         dUc = (Uc - Uc_ant)/T_amos;
% % % %         Uc_ant = Uc;
% % % %         
% % % %         A.pSC.U = F\A.pPos.X([7 8 9 12]);
% % % %         
% % % %         % Controlador Adaptativo
% % % %         g11 = dUc(1) + Kd(1)*(Uc(1) - A.pSC.U(1));
% % % %         g15 = A.pSC.U(1);
% % % %         g22 = dUc(2) + Kd(2)*(Uc(2) - A.pSC.U(2));
% % % %         g26 = A.pSC.U(2);
% % % %         g33 = dUc(3) + Kd(3)*(Uc(3) - A.pSC.U(3));
% % % %         g37 = A.pSC.U(3);
% % % %         g44 = dUc(4) + Kd(4)*(Uc(4) - A.pSC.U(4));
% % % %         g48 = A.pSC.U(4);
% % % %         
% % % %         G = [ g11   0   0   0   g15   0   0   0 ;
% % % %             0   g22   0   0   0   g26   0   0  ;
% % % %             0   0   g33   0   0   0   g37   0  ;
% % % %             0   0   0   g44   0   0   0   g48 ];
% % % %         
% % % %         D = diag([THETA(1) THETA(2) THETA(3) THETA(4)]);
% % % %         SIGMA = dUc + Kd*(Uc - A.pSC.U);
% % % %         NETA = diag(A.pSC.U)*...
% % % %             [THETA(5); THETA(6); THETA(7); THETA(8)];
% % % %         
% % % %         if toc(t) < T_aprend
% % % %             dTHETA_est = 0;
% % % %         else
% % % %             dTHETA_est = GAMA\G'*(Uc - A.pSC.U);
% % % %         end
% % % %         
% % % %         THETA_est = THETA_est + dTHETA_est*T_amos;
% % % %         THETA_error = THETA_est - THETA;
% % % %         
% % % %         A.pSC.Ud = D*SIGMA + NETA + G*THETA_error;
        
        
       
%         A.pSC.Ud = Uc;
        
%         A = J.mControl(A);           % joystick command (priority)
        A.rSendControlSignals;
        
        H = [H; [A.pPos.Xd' A.pPos.X' toc(t)]];
      
        if toc(t_plot) > T_plot
            t_plot = tic;
            try %#ok<TRYNC>
                delete (fig);
            end
            A.mCADplot
            fig(1) = plot3(H(:,1),H(:,2),H(:,3),'r--','LineWidth',2);
            fig(2) = plot3(H(:,13),H(:,14),H(:,15),'b','LineWidth',2);
            drawnow;
        end
        
        
    end
end


% Land drone
if A.pFlag.Connected == 1
    A.rLand;
    %     A.rDisconnect;
end

