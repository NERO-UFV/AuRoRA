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
    RI.rConnect('192.168.0.100');
%     B = Bebop(1,'B');
    
%     P = Pioneer3DX(1);  % Pioneer Instance
    P = RPioneer(1,'RosAria',1);
%     P2 = Pioneer3DX(1);
%     P3 = Pioneer3DX(2);
    
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

idP = 1;
idT1 = 2;
idT2 = 3;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
%
rb = OPT.RigidBody;          % read optitrack data
P = getOptData(rb(idP),P);   % get pioneer data
% P2 = getOptData(rb(idT1),P2);   % get pioneer data
% P3 = getOptData(rb(idT2),P3);   % get pioneer data

P.pPos.X(1:6)
% P2.pPos.X(1:6)
% P3.pPos.X(1:6)

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

rX = 2.2;           % [m]
rY = 2;           % [m]
T = T_MAX;             % [s]
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
angle1OPT_hist = [];
angle2OPT_hist = [];
t_hist = [];

try
    while toc(t) < T_MAX
        
        if toc(t_control) > T_CONTROL
            
            t_control = tic;
            t_atual = toc(t);
%% POSIÇÃO            

            
            % Dados Odometria
%             P.rGetSensorData;
            P.rGetPotentiometerData;
            rb = OPT.RigidBody; 
            P = getOptData(rb(idP),P);   % get pioneer data
%             P2 = getOptData(rb(idT1),P2);   % get pioneer data
%             P3 = getOptData(rb(idT2),P3);   % get pioneer data

%             
%             P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual);rY*cos(ww*t_atual)];
%             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual);-ww*rY*sin(ww*t_atual)];
% 
%             P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual + phase);rY*cos(ww*t_atual + phase)-rY];
%             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual + phase);-ww*rY*sin(ww*t_atual + phase)];
            
%             P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual);rY*cos(ww*t_atual)];
%             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual);-ww*rY*sin(ww*t_atual)];
            
            P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual);rY*sin(2*ww*t_atual)];
            P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual);2*ww*rY*cos(2*ww*t_atual)];

            
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
            
          K = [ cos(P.pPos.X(6)), -P.pPar.a*sin(P.pPos.X(6)); ...
                sin(P.pPos.X(6)), +P.pPar.a*cos(P.pPos.X(6))];
            
          P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
            
          P.pSC.Ud(1:2) = K\(P.pPos.Xd(7:8) + Kp1*tanh(Kp2*P.pPos.Xtil(1:2)));
          
%           P.pSC.Ud = [0.1 0]';
          P = J.mControl(P);                    % joystick command (priority)
%               disp('angle 1')
%               P.pPot1.Data
%               disp('angle 2')
%               P.pPot2.Data

%             B.pSC.Ud
%             P.pSC.Ud
          P.rCommand;
          
          xd_hist = [xd_hist P.pPos.Xd(1:6)];
          x_hist = [x_hist P.pPos.X(1:6)];
          angle1_hist = [angle1_hist P.pPot1.Data];
          angle2_hist = [angle2_hist P.pPot2.Data];
%           angle1OPT_hist = [angle1OPT_hist P2.pPos.X(6)];
%           angle2OPT_hist = [angle2OPT_hist P3.pPos.X(6)];
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
%%
figure();
hold on;
grid on;
plot(xd_hist(1,:),xd_hist(2,:));
plot(x_hist(1,:),x_hist(2,:));
title('XY');
xlabel('X [m]');
ylabel('Y [m]');

figure();
hold on;
grid on;
plot(t_hist,angle1_hist);
plot(t_hist,angle2_hist);
title('\theta Pot');
legend('\theta_1','\theta_2')
xlabel('t [s]');
ylabel('angulo [°]');

% angle1OPT_hist_ret = x_hist(6,:) - angle1OPT_hist;
% angle2OPT_hist_ret = angle1OPT_hist - angle2OPT_hist;
% 
% for i = 1:length(angle1OPT_hist_ret)
%     if abs(angle1OPT_hist_ret(i)) > pi
%         angle1OPT_hist_ret(i) = angle1OPT_hist_ret(i) - 2*pi*sign(angle1OPT_hist_ret(i));
%     end
%     if abs(angle2OPT_hist_ret(i)) > pi
%         angle2OPT_hist_ret(i) = angle2OPT_hist_ret(i) - 2*pi*sign(angle2OPT_hist_ret(i));
%     end
% end
% 
% angle1OPT_hist_ret = angle1OPT_hist_ret*180/pi;
% angle2OPT_hist_ret = angle2OPT_hist_ret*180/pi;
% 
% figure();
% hold on;
% grid on;
% plot(t_hist,angle1OPT_hist_ret);
% plot(t_hist,angle2OPT_hist_ret);
% title('\theta Opt');
% legend('\theta_1','\theta_2')
% xlabel('t [s]');
% ylabel('angulo [°]');
% 
% figure();
% hold on;
% grid on;
% plot(t_hist,angle1_hist);
% plot(t_hist,angle1OPT_hist_ret);
% title('\theta_1 Comparação');
% legend('\theta_1 (Pot)','\theta_1 (Opt)')
% xlabel('t [s]');
% ylabel('angulo [°]');
% 
% figure();
% hold on;
% grid on;
% plot(t_hist,angle2_hist);
% plot(t_hist,angle2OPT_hist_ret);
% title('\theta_2 Comparação');
% legend('\theta_2 (Pot)','\theta_2 (Opt)')
% xlabel('t [s]');
% ylabel('angulo [°]');
% 
% %%
path = [pwd '\ROS\Scripts\DiegoCBA\'];
filename = ['CBADiego_Pot_Opt_' datestr(now,30) '_T' num2str(T) '.mat'];
fullname = [path filename];
save(fullname,'xd_hist','x_hist','angle1_hist','angle2_hist','t_hist')
