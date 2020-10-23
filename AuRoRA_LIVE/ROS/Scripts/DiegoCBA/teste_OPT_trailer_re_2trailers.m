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
    setenv('ROS_MASTER_URI','http://192.168.0.103:11311')
    setenv('ROS_IP','192.168.0.104')
    RI = RosInterface;
    RI.rConnect('192.168.0.103');
%     B = Bebop(1,'B');
    
%     P = Pioneer3DX(1);  % Pioneer Instance
    P = RPioneer(1,'RosAria',2);
    P2 = Pioneer3DX(1);
    P3 = Pioneer3DX(2);
    
    % Joystick
    J = JoyControl;

    

    
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

idP = 1;
idT1 = 2;
idT2 = 3;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
%
rb = OPT.RigidBody;          % read optitrack data
P = getOptData(rb(idP),P);   % get pioneer data
P2 = getOptData(rb(idT1),P2);   % get pioneer data
P3 = getOptData(rb(idT2),P3);   % get pioneer data

% P.pPos.X(1:6)
% P2.pPos.X(1:6)
% P3.pPos.X(1:6)

% Time variables initialization
T_CONTROL = 1/10; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 150;

fprintf('\nStart..............\n\n');

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;

rX = 1.5;           % [m]
rY = 1.5;           % [m]
T = T_MAX;             % [s]
Tf = T*2;            % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;
cont = 0;

pgains = [0.35 0.35 0.5 0.5];
Kp1 = diag([pgains(1), pgains(2)]);
Kp2 = diag([pgains(3), pgains(4)]);

Kp1 = 0.6;
Kp2 = 0.5;

xd_hist = [];
x_hist = [];
angle1_hist = [];
angle2_hist = [];
angle1OPT_hist = [];
angle2OPT_hist = [];
t_hist = [];
x_t1_hist = [];
controlT0_hist = [];
controlT1_hist = [];
P.pPar.a = 0.205;
L0 = 0.3;
L1 = 0.455;
L10 = 0.28;
L2 = 0.455;
alpha = pi;
b = 0.2;

try
    while toc(t) < T_MAX
        
        if toc(t_control) > T_CONTROL
            
            t_control = tic;
            t_atual = toc(t);

            P.rGetPotentiometerData;
            rb = OPT.RigidBody; 
            P = getOptData(rb(idP),P);   % get pioneer data
            P2 = getOptData(rb(idT1),P2);   % get trailer 1
            P3 = getOptData(rb(idT2),P3);   % get trailer 2

%             
%             P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual);rY*cos(ww*t_atual)];
%             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual);-ww*rY*sin(ww*t_atual)];
% 
%             P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual + phase);rY*cos(ww*t_atual + phase)-rY];
%             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual + phase);-ww*rY*sin(ww*t_atual + phase)];
            
%             P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual);rY*cos(ww*t_atual)];
%             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual);-ww*rY*sin(ww*t_atual)];
            
            P3.pPos.Xd([1 2]) = [-rX*sin(ww*t_atual + phase);-rY*cos(ww*t_atual + phase)];
            P3.pPos.Xd([7 8]) = [-ww*rX*cos(ww*t_atual + phase);ww*rY*sin(ww*t_atual + phase)];
            
%             P3.pPos.Xd([1 2]) = [-rX*sin(ww*t_atual + phase);-rY*sin(2*ww*t_atual + phase)];
%             P3.pPos.Xd([7 8]) = [-ww*rX*cos(ww*t_atual + phase);-2*ww*rY*cos(2*ww*t_atual + phase)];

            
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
            
          %%%% angulo relativo
%           P2.pPos.X(6) = P2.pPos.X(6) + 10*pi/180;
          theta_1 = P.pPos.X(6) - P2.pPos.X(6);
          theta_2 = P2.pPos.X(6) - P3.pPos.X(6);
            
          %%%%%%%%%%%%%%%%%
            
%           K = [ cos(P.pPos.X(6)), -P.pPar.a*sin(P.pPos.X(6)); ...
%                 sin(P.pPos.X(6)), +P.pPar.a*cos(P.pPos.X(6))];

%           K = [ cos(P2.pPos.X(6)), -P.pPar.a*sin(P2.pPos.X(6)); ...
%                 sin(P2.pPos.X(6)), +P.pPar.a*cos(P2.pPos.X(6))];
            
%           pose_final_trailer = P2.pPos.Xc - [b*cos(P2.pPos.X(6)); b*sin(P2.pPos.X(6))];  

            
          P3.pPos.Xtil = P3.pPos.Xd - P3.pPos.Xc;
          
          angulo_trailer_mundo = P3.pPos.X(6);
          
          
          K_inv = [ cos(angulo_trailer_mundo), sin(angulo_trailer_mundo), b*sin(alpha); ...
                -sin(angulo_trailer_mundo), cos(angulo_trailer_mundo), -b*cos(alpha); ...
                0, 0, 1];
            
          vx = (P3.pPos.Xd(7) + Kp1*tanh(Kp2*P3.pPos.Xtil(1)));
          vy = (P3.pPos.Xd(8) + Kp1*tanh(Kp2*P3.pPos.Xtil(2)));
          v_fi = (-vx/(b*cos(alpha)))*sin(angulo_trailer_mundo) +  (vy/(b*cos(alpha)))*cos(angulo_trailer_mundo);
            
          sinal_controle_trailer = K_inv*[vx; ...
                                          vy;...
                                          v_fi];
          
    
          v2 = sinal_controle_trailer(1);
          w2 = sinal_controle_trailer(3);
          
          v1 = v2*cos(theta_2) + (w2)*(L2)*sin(theta_2);
          w1 = v2*sin(theta_2)/L10 - (w2)*(L2)*cos(theta_2)/L10;
          
          v0_c = v1*cos(theta_1) + (w1)*(L1)*sin(theta_1);
          w0_c = v1*sin(theta_1)/L0 - (w1)*(L1)*cos(theta_1)/L0;
          
          P.pSC.Ud(1:2) = [v0_c; w0_c];
          
          P = J.mControl(P);                    % joystick command (priority)

%           P.rCommand;
          
          xd_hist = [xd_hist P2.pPos.Xd(1:6)];
          x_hist = [x_hist P.pPos.X(1:6)];
          angle1_hist = [angle1_hist P.pPot1.Data];
%           angle2_hist = [angle2_hist P.pPot2.Data];
          angle1OPT_hist = [angle1OPT_hist P2.pPos.X(6)];
%           angle2OPT_hist = [angle2OPT_hist P3.pPos.X(6)];
          t_hist = [t_hist toc(t)];
          x_t1_hist = [x_t1_hist P2.pPos.Xc(1:6)];
          controlT0_hist = [controlT0_hist [v0_c w0_c]'];
          controlT1_hist = [controlT1_hist [v1 w1]'];
                     
          
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

%%
figure();
hold on;
grid on;
plot(xd_hist(1,:),xd_hist(2,:),'b');
plot(x_t1_hist(1,:),x_t1_hist(2,:),'r');
% plot(x_hist(1,:),x_hist(2,:),'k');
legend('Desejado','Frente Trailer (controle)', 'Centro robo');
xlabel('X [m]');
ylabel('Y [m]');

figure();
hold on;
grid on;
plot(t_hist(1,:), controlT1_hist(1,:));
plot(t_hist(1,:), controlT1_hist(2,:));
legend('sinal controle v1', 'sinal controle w1');
title('sinais controle');
xlabel('t');
ylabel('sinal');

figure();
hold on;
grid on;
plot(t_hist(1,:), controlT0_hist(1,:));
plot(t_hist(1,:), controlT0_hist(2,:));
legend('sinal controle v0', 'sinal controle w0');
title('sinais controle');
xlabel('t');
ylabel('sinal');

figure();
hold on;
grid on;
plot(t_hist(1,:), xd_hist(1,:) - x_t1_hist(1,:));
plot(t_hist(1,:), xd_hist(2,:) - x_t1_hist(2,:));
legend('erro trailer x', 'erro trailer y');
title('erro posições');
xlabel('t');
ylabel('sinal');


%%

path = [pwd '\ROS\Scripts\DiegoCBA\'];
filename = ['Diego_Opt_T_' num2str(T) '_Data_' datestr(now,30) '.mat'];
fullname = [path filename];
% save(fullname,'xd_hist','x_t1_hist','controlT1_hist','controlT0_hist','t_hist')

