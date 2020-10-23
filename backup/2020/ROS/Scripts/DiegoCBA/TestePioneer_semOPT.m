clear; close all; clc;
try
    fclose(instrfindall);
catch
end
%
% % Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Classes

%% Load Class
try
    % Load Classes
    RI = RosInterface;
    RI.rConnect('192.168.0.103');
%     B = Bebop(1,'B');
    
%     P = Pioneer3DX(1);  % Pioneer Instance
    P = RPioneer(1,'RosAria',1);
    
    % Joystick
    J = JoyControl;
    
    % Create OptiTrack object and initialize
%     OPT = OptiTrack;
%     OPT.Initialize;
%     idB = getID(OPT,B); % ID do Bebop
    

    
    disp('################### Load Class Success #######################');
    
catch ME
    disp(' ');
    disp(' ################### Load Class Issues #######################');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    rosshutdown;
    return;
    
end

%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);

%% Variable initialization
data = [];

% Time variables initialization
T_CONTROL = 1/10; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 60;

Xd_i = 1;
Xd(:,1) = [0.5 0 0 0]';

P.pPos.Xd(1:3) = Xd(1:3,Xd_i);
P.pPos.Xd(6) = Xd(4,Xd_i);

fprintf('\nStart..............\n\n');

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;

rX = 1;           % [m]
rY = 1;           % [m]
T = 60;             % [s]
Tf = T*2;            % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;
cont = 0;

pgains = [0.35 0.35 0.8 0.8];
Kp1 = diag([pgains(1), pgains(2)]);
Kp2 = diag([pgains(3), pgains(4)]);

xd_hist = [];
x_hist = [];
angle1_hist = [];
angle2_hist = [];
t_hist = [];

try
    while toc(t) < T_MAX
        
        if toc(t_control) > T_CONTROL
            
            t_control = tic;
            t_atual = toc(t);
%% POSIÇÃO            
            
            % Dados Odometria
            P.rGetSensorData;
%             P.rGetPotentiometerData;
%             
%             P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual);rY*cos(ww*t_atual)];
%             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual);-ww*rY*sin(ww*t_atual)];

%               X_atras = P.pPos.Xc(1:2) - [P.pPar.a*cos(P.pPos.X(6)); P.pPar.a*sin(P.pPos.X(6))];

%             P.pPos.X(6) = P.pPos.X(6) + pi;
%             
%             if abs(P.pPos.X(6)) > pi
%                 P.pPos.X(6) = P.pPos.X(6) - 2*pi*sign(P.pPos.X(6));
%             end
%             
%             P.pPos.X([1:2]) = [-1 0;0 -1]*P.pPos.X([1:2]);




%             P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual + phase);-rY*cos(ww*t_atual + phase)+rY];
%             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual + phase);ww*rY*sin(ww*t_atual + phase)];
            
            P.pPos.Xd([1 2]) = [-rX*sin(ww*t_atual + phase)+P.pPar.a;-rY*cos(ww*t_atual + phase)+rY];
            P.pPos.Xd([7 8]) = [-ww*rX*cos(ww*t_atual + phase);ww*rY*sin(ww*t_atual + phase)];

%             P.pPos.Xd([1 2]) = [-rX*sin(ww*t_atual + phase);rY*cos(ww*t_atual + phase)-rY];
%             P.pPos.Xd([7 8]) = [-ww*rX*cos(ww*t_atual + phase);-ww*rY*sin(ww*t_atual + phase)];
            
%             P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual);rY*sin(2*ww*t_atual)];
%             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual);2*ww*rY*cos(2*ww*t_atual)];

            
            %% Save data
            
            % Variable to feed plotResults function
%             data = [  data  ; B.pPos.Xd'     B.pPos.X'        B.pSC.Ud'         B.pSC.U' ...
%                       t_atual B.pPar.Model_simp'];
            
            % %         %   1 -- 12      13 -- 24     25 -- 29          30 -- 34
            % %             P.pPos.Xd'   P.pPos.X'    P.pSC.Ud(1:2)'    P.pSC.U(1:2)'
            % %
            % %         %   29 -- 40     41 -- 52     53 -- 56          57 -- 60
            % %             B.pPos.Xd'   B.pPos.X'    B.pSC.Ud'         B.pSC.U'
            % %
            % %         %   61 -- 66     67 -- 72       73 -- 78       79
            % %             LF.pPos.Qd'  LF.pPos.Qtil'  LF.pPos.Xd'    toc(t)  ];
            
            
            % Beboop
            % Joystick Command Priority
            
%           K = [cos(P.pPos.X(6)), -P.pPar.a*sin(P.pPos.X(6)); ...
%                sin(P.pPos.X(6)), P.pPar.a*cos(P.pPos.X(6))];
           
          K = [cos(P.pPos.X(6)), -P.pPar.a*sin(P.pPos.X(6)); ...
               sin(P.pPos.X(6)), P.pPar.a*cos(P.pPos.X(6))];
            
            
            
          P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
%           P.pPos.Xtil = P.pPos.Xd(1:2) - X_atras;
            
          P.pSC.Ud(1:2) = K\(P.pPos.Xd(7:8) + Kp1*tanh(Kp2*P.pPos.Xtil(1:2)));
%           P.pSC.Ud(1:2) = [-0.2; -0.2];
%           P.pSC.Ud(1:2) = -P.pSC.Ud(1:2);
 
          
%           P.pSC.Ud = [0.1 0]';
          P = J.mControl(P);                    % joystick command (priority)
%               disp('angle 1')
%               P.pPot1.Data
%               disp('angle 2')
%               P.pPot2.Data

%             B.pSC.Ud
          P.rCommand;
          
          xd_hist = [xd_hist P.pPos.Xd(1:2)];
          x_hist = [x_hist P.pPos.X(1:2)];
%           angle1_hist = [angle1_hist P.pPot1.Data];
%           angle2_hist = [angle2_hist P.pPot2.Data];
          t_hist = [t_hist toc(t)];
            
            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            drawnow
            if btnEmergencia ~= 0 
                disp('Bebop Landing through Emergency Command ');

                % Send 3 times Commands 1 second delay to Drone Land
                
                break;
            end
            
            
        end
    end
catch ME
    
    disp('Bebop Landing through Try/Catch Loop Command');
    P.rCmdStop;
    
end


% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

figure();
hold on;
grid on;
plot(xd_hist(1,:),xd_hist(2,:));
plot(x_hist(1,:),x_hist(2,:));
legend('desejado','real')
xlabel('X [m]');
ylabel('Y [m]');

% figure();
% hold on;
% grid on;
% plot(t_hist,angle1_hist);
% %plot(t_hist,angle2_hist);
% % title('XY');
% legend('\theta_1','\theta_2')
% xlabel('t [s]');
% ylabel('angulo [°]');

%% Plot results
% Xtil = data(:,1:12) - data(:,13:24);
% 
% figure();
% hold on;
% grid on;
% plot(data(:,35),Xtil(:,1));
% plot(data(:,35),Xtil(:,2));
% plot(data(:,35),Xtil(:,3));
% plot(data(:,35),Xtil(:,6));
% title('Erro de Posição');
% legend('Pos X','Pos Y','Pos Z', 'Ori Z');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');
% 
% figure();
% hold on;
% grid on;
% plot(data(:,1),data(:,2));
% plot(data(:,13),data(:,14));
% title('XY');
% xlabel('X [m]');
% ylabel('Y [m]');
% 
% figure();
% subplot(411)
% hold on;
% grid on;
% plot(data(:,35),data(:,7));
% plot(data(:,35),data(:,19));
% xlabel('Tempo[s]');
% ylabel('Velocidade [m/s]');
% legend('dX', 'dXd');
% 
% subplot(412)
% hold on;
% grid on;
% plot(data(:,35),data(:,8));
% plot(data(:,35),data(:,20));
% xlabel('Tempo[s]');
% ylabel('Velocidade [m/s]');
% legend('dY', 'dYd');
% 
% subplot(413)
% hold on;
% grid on;
% plot(data(:,35),data(:,9));
% plot(data(:,35),data(:,21));
% xlabel('Tempo[s]');
% ylabel('Velocidade [m/s]');
% legend('dZ', 'dZd');
% 
% subplot(414)
% hold on;
% grid on;
% plot(data(:,35),data(:,18));
% plot(data(:,35),data(:,24));
% xlabel('Tempo[s]');
% ylabel('Velocidade [rad/s]');
% legend('phi', 'dphi');
% 
% subplot(421)
% grid on;
% plot(data(:,35),data(:,36));
% xlabel('Tempo[s]');
% ylabel('$K_1$','interpreter','latex')
% legend(['$K_1 = $' num2str(data(end,36))],'interpreter','latex')
% 
% subplot(422)
% grid on;
% plot(data(:,35),data(:,37));
% xlabel('Tempo[s]');
% ylabel('$K_2$','interpreter','latex')
% legend(['$K_2 = $' num2str(data(end,37))],'interpreter','latex')
% 
% subplot(423)
% grid on;
% plot(data(:,35),data(:,38));
% xlabel('Tempo[s]');
% ylabel('$K_3$','interpreter','latex')
% legend(['$K_3 = $' num2str(data(end,38))],'interpreter','latex')
% 
% subplot(424)
% grid on;
% plot(data(:,35),data(:,39));
% xlabel('Tempo[s]');
% ylabel('$K_4$','interpreter','latex')
% legend(['$K_4 = $' num2str(data(end,39))],'interpreter','latex')
% 
% subplot(425)
% grid on;
% plot(data(:,35),data(:,40));
% xlabel('Tempo[s]');
% ylabel('$K_5$','interpreter','latex')
% legend(['$K_5 = $' num2str(data(end,40))],'interpreter','latex')
% 
% subplot(426)
% hold on;
% grid on;
% plot(data(:,35),data(:,41));
% xlabel('Tempo[s]');
% ylabel('$K_6$','interpreter','latex')
% legend(['$K_6 = $' num2str(data(end,41))],'interpreter','latex')
% 
% subplot(427)
% hold on;
% grid on;
% plot(data(:,35),data(:,42));
% xlabel('Tempo[s]');
% ylabel('$K_7$','interpreter','latex')
% legend(['$K_7 = $' num2str(data(end,42))],'interpreter','latex')
% 
% subplot(428)
% hold on;
% grid on;
% plot(data(:,35),data(:,43));
% xlabel('Tempo[s]');
% ylabel('$K_8$','interpreter','latex')
% legend(['$K_8 = $' num2str(data(end,43))],'interpreter','latex')
