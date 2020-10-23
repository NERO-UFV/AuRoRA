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
t_int = tic;
t_derivada = tic;

rX = 1.5;           % [m]
rY = 1.5;           % [m]
T = T_MAX;             % [s]
Tf = T*2;            % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;
cont = 0;
L0 = 0.3;
L1 = 0.455;
P.pPar.a = 0.19;
Xda = 0;

pgains = [0.3 0.3 0.5 0.5];
Kp1 = diag([pgains(1), pgains(2)]);
Kp2 = diag([pgains(3), pgains(4)]);

Kp1 = 0.6;
Kp2 = 0.5;

xd_hist = [];
x_hist = [];
angle1_hist = [];
angle2_hist = [];
t_hist = [];
t1_hist = [];
vel_hist = [];
t0_c_hist = [];
t1_c_hist = [];
t0_ct_hist = [];
pose_final_trailer_hist = [];
centro_robo_hist = [];
control_hist = [];
theta_eq_hist = [];
controlT_hist = [];
w0_c_hist = [];
x_atras_hist = [];
centro_trailer_hist = [];
theta1_giro = 25;
a = 0.15;
b = 0.2;
alpha = pi;
% z = 0.4;

%try
    while toc(t) < T_MAX
        
        if toc(t_control) > T_CONTROL
            
            t_control = tic;
            t_atual = toc(t);
%% POSIÇÃO            

            
            % Dados Odometria
            P.rGetSensorData;
            P.rGetPotentiometerData;
            
            
            %%% -----Mudança referencial - robo trajetoria
%             P.pPos.X(6) = P.pPos.X(6) + pi;
%             
%             if abs(P.pPos.X(6)) > pi
%                 P.pPos.X(6) = P.pPos.X(6) - 2*pi*sign(P.pPos.X(6));
%             end
%             
%             P.pPos.X([1:2]) = [-1 0;0 -1]*P.pPos.X([1:2]);
%             P.pPos.Xc([1:2]) = [-1 0;0 -1]*P.pPos.Xc([1:2]);

            r = 1;
            vd = 2*pi*r/T;
            wd = vd/r;

            
%             
% % % %             P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual);rY*cos(ww*t_atual)];
% % % %             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual);-ww*rY*sin(ww*t_atual)];
            P.pPos.Xd([1 2]) = [-rX*sin(ww*t_atual + phase) - (L0+L1+b);-rY*cos(ww*t_atual + phase)+rY];
            P.pPos.Xd([7 8]) = [-ww*rX*cos(ww*t_atual + phase);ww*rY*sin(ww*t_atual + phase)];

%             P.pPos.Xd([1 2]) = [-rX*sin(ww*t_atual + phase) - (L0+L1+b);-rY*sin(2*ww*t_atual + phase)];
%             P.pPos.Xd([7 8]) = [-ww*rX*cos(ww*t_atual + phase);-2*ww*rY*cos(2*ww*t_atual + phase)];
            
%             P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual + phase);-rY*cos(ww*t_atual + phase)+rY];
%             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual + phase);ww*rY*sin(ww*t_atual + phase)];

%             P.pPos.Xd(6) = P.pPos.Xd(6) + P.pPos.Xd(12)*toc(t_int);
%             t_int = tic;
%             P.pPos.Xd(12) = wd;

%%%% trajetoria para tras
            
%             P.pPos.Xd([1 2]) = [-(rX*sin(ww*t_atual + phase)-(L0+L1-P.pPar.a));-rY*cos(ww*t_atual + phase)+rY];
%             P.pPos.Xd([7 8]) = [-ww*rX*cos(ww*t_atual + phase);ww*rY*sin(ww*t_atual + phase)];
            
% % % % %             P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual);rY*sin(2*ww*t_atual)];
% % % % %             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual);2*ww*rY*cos(2*ww*t_atual)];

            
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
            
            

          %%% -- Pose ponto final do Trailer (assumi sinal negativo para theta1 - potenciomero em relação
          %%% ao mundo (análise de giro do pioneer somente));
          
%           y_t = P.pPos.Xc(2) + L0*sin(P.pPos.X(6)) + (L1-P.pPar.a)*sin(P.pPos.X(6) - P.pPot1.Data*pi/180);
%           x_t = P.pPos.Xc(1) - L0*cos(P.pPos.X(6)) - (L1-P.pPar.a)*cos(P.pPos.X(6) - P.pPot1.Data*pi/180);

          x_t = P.pPos.Xc(1) - L0*cos(P.pPos.X(6)) - (L1+b)*cos(P.pPos.X(6) - P.pPot1.Data*pi/180);
          y_t = P.pPos.Xc(2) - L0*sin(P.pPos.X(6)) - (L1+b)*sin(P.pPos.X(6) - P.pPot1.Data*pi/180);
          
%           x_tc = P.pPos.Xc(1) - L0*cos(P.pPos.X(6)) - (L1)*cos(P.pPos.X(6) - P.pPot1.Data*pi/180);
%           y_tc = P.pPos.Xc(2) - L0*sin(P.pPos.X(6)) - (L1)*sin(P.pPos.X(6) - P.pPot1.Data*pi/180);
          
          pose_final_trailer = [x_t; y_t];

%           pose_centro_trailer = [x_t; y_t];
          
          %%% -------------------------------------------------------------
          
          centro_robo = [P.pPos.Xc(1); P.pPos.Xc(2)];
          
%         X_atras = P.pPos.Xc(1:2) - [P.pPar.a*cos(P.pPos.X(6)); P.pPar.a*sin(P.pPos.X(6))];



          %%% ---------- erro em relacao ao ponto final do trailer
         
%          P.pPos.Xtil = P.pPos.Xd(1:2) - X_atras;
%          P.pPos.Xtil = P.pPos.Xd(1:2) - P.pPos.Xc(1:2);
           P.pPos.Xtil = P.pPos.Xd(1:2) - pose_final_trailer;
          
          %%% ---------------------------------------------------------
          
          %%% ----- Trailer como Pioneer - modelo direto e controle para obter v1 e w1
          %%% neccesarios para seguimento de trajetória;
          
          angulo_trailer_mundo = P.pPos.Xc(6) - (P.pPot1.Data*pi/180);
%           angulo_trailer_mundo = P.pPos.X(6);
          
%           K = [ cos(angulo_trailer_mundo), -P.pPar.a*sin(angulo_trailer_mundo); ...
%                 sin(angulo_trailer_mundo), +P.pPar.a*cos(angulo_trailer_mundo)];    

          K_inv = [ cos(angulo_trailer_mundo), sin(angulo_trailer_mundo), b*sin(alpha); ...
                -sin(angulo_trailer_mundo), cos(angulo_trailer_mundo), -b*cos(alpha); ...
                0, 0, 1];
            
                
          vx = (P.pPos.Xd(7) + Kp1*tanh(Kp2*P.pPos.Xtil(1)));
          vy = (P.pPos.Xd(8) + Kp1*tanh(Kp2*P.pPos.Xtil(2)));
          
          
%           P.pPos.Xd(6) = atan2(vy,vx);
          
%           erro_angulo = P.pPos.Xd(6) - angulo_trailer_mundo;
%           
%           if abs(erro_angulo) > pi
%               erro_angulo = erro_angulo - 2*pi*sign(erro_angulo);
%           end
%           
%           wd = (P.pPos.Xd(6) - Xda)/toc(t_derivada);
%           
%           if wd > 1
%               wd = 1;
%           elseif wd < -1
%               wd = -1;
%           end
%              
%           t_derivada = tic;
%           Xda = P.pPos.Xd(6);
                    
%           v_fi = (wd + Kp2*tanh(Kp1*(erro_angulo)));

          v_fi = (-vx/(b*cos(alpha)))*sin(angulo_trailer_mundo) +  (vy/(b*cos(alpha)))*cos(angulo_trailer_mundo);
          
          sinal_controle_trailer = K_inv*[vx; ...
                                          vy;...
                                          v_fi];

      
%           sinal_controle_trailer = K\(P.pPos.Xd(7:8) + Kp1*tanh(Kp2*P.pPos.Xtil(1:2)));
          
%           sinal_controle_trailer = [(P.pPos.Xd(7) + Kp1*tanh(Kp2*P.pPos.Xtil(1)))*cos(angulo_trailer_mundo)+(P.pPos.Xd(8) + Kp1*tanh(Kp2*P.pPos.Xtil(2)))*sin(angulo_trailer_mundo)+a*(wd + Kp2*tanh(Kp1*(theta1_giro*pi/180))); (wd + Kp2*tanh(Kp1*(theta1_giro*pi/180)))]
                    
          %%% -----Modelo inverso - transforma sinais de velocidades do
          %%% trailer para o Pioneer
          
          v1 = sinal_controle_trailer(1);
          w1 = sinal_controle_trailer(3);
          
          v0_c = v1*cos(P.pPot1.Data*pi/180) + (w1)*(L1)*sin(P.pPot1.Data*pi/180);
          w0_c = -v1*sin(P.pPot1.Data*pi/180)/L0 + (w1)*(L1)*cos(P.pPot1.Data*pi/180)/L0;

%           v0_c = v1*cos(P.pPot1.Data*pi/180) + (w1)*(L1)*sin(P.pPot1.Data*pi/180)
%           w0_c = -v1*sin(P.pPot1.Data*pi/180)/L1 + (w1)*cos(P.pPot1.Data*pi/180)

%           t0_c =  [v0_c;w0_c];
%           signal_control = (w0_c/v0_c);
          
%           %%%% ---- calculo raio de giro limite
%           
%           fi_1 = (w1 - P.pPos.X(12))/v1;
%           fi_0 = P.pPos.X(12)/v0_c;
%           
%           %%% --- calculo equilibrio
%           
%           theta_eq = -atan(fi_0*L0)-atan(fi_1*L1);
%          
%           %%%% -----limite
%           
%           fi_m1 = 1/(sqrt(L1^2 - L0^2));
%           fi_m2 = abs(sin(pi/6)/L1+L0*cos(pi/6));
%           fi_m3 = fi_1/(sqrt(1+((fi_1)^2)*(L1^2-L0^2)));
%           fi_limite = min([fi_m1 fi_m2 fi_m3]);
          
          %%%% ---- sinal de controle enviado ao robo
          
%           if signal_control > fi_limite
%               signal_control = fi_limite;
%               v0_c = w0_c/fi_limite;
%               w0_c = v0_c*fi_limite;
%               P.pSC.Ud(1:2) = [0.4*v0_c; 0.4*w0_c];
%           else
%               P.pSC.Ud(1:2) = [v0_c; w0_c];
%           end
          
          
          P.pSC.Ud(1:2) = [v0_c; -w0_c];
%           P.pSC.Ud(1:2) = [v1; w1];
          %%%% ---- sinal de controle original
          
%           P.pSC.Ud(1:2) = [v0_c; w0_c];
%           
          %%%---------

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
%           t1_hist = [t1_hist t1];
%           vel_hist = [vel_hist vel];
%           control_hist = [control_hist P.pSC.Ud(1:2)];
%           t0_c_hist = [t0_c_hist t0_c];
          pose_final_trailer_hist = [pose_final_trailer_hist pose_final_trailer];
          centro_robo_hist = [centro_robo_hist centro_robo];
%           theta_eq_hist = [theta_eq_hist theta_eq];
          controlT_hist = [controlT_hist [v1 w1]'];
          w0_c_hist = [w0_c_hist [-v1*sin(P.pPot1.Data*pi/180)/L0  -(-w1)*(L1)*cos(P.pPot1.Data*pi/180)/L0]' ];
%           centro_trailer_hist = [centro_trailer_hist pose_centro_trailer];
%           x_atras_hist = [x_atras_hist X_atras];
            
            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            drawnow
            if btnEmergencia ~= 0 
                disp('Bebop Landing through Emergency Command ');

                % Send 3 times Commands 1 second delay to Drone Land
                
                break;
            end
            
            
        end
    end
%catch ME
    
%     disp('Bebop Landing through Try/Catch Loop Command');
%     P.rCmdStop;
%     
% end


% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");
%%
% figure();
% hold on;
% grid on;
% plot(xd_hist(1,:),xd_hist(2,:));
% plot(x_hist(1,:),x_hist(2,:));
% title('XY');
% xlabel('X [m]');
% ylabel('Y [m]');

% figure();
% hold on;
% grid on;
% plot(t_hist(1,:),t0_c_hist(1,:));
% legend('v');
% title('v');
% xlabel('t');
% ylabel('v');
% 
% figure();
% hold on;
% grid on;
% plot(t_hist(1,:),(180/pi)*t0_c_hist(2,:));
% legend('w');
% title('omega');
% xlabel('t');
% ylabel('w');

% figure();
% hold on;
% grid on;
% plot(t_hist(1,:), control_hist(1,:));
% plot(t_hist(1,:), control_hist(2,:));
% legend('sinal controle v0', 'sinal controle w0');
% title('sinais controle');
% xlabel('t');
% ylabel('sinal');


figure();
hold on;
grid on;
plot(t_hist(1,:), controlT_hist(1,:));
plot(t_hist(1,:), controlT_hist(2,:));
legend('sinal controle v1', 'sinal controle w1');
title('sinais controle');
xlabel('t');
ylabel('sinal');

figure();
hold on;
grid on;
plot(t_hist(1,:), w0_c_hist(1,:));
plot(t_hist(1,:), w0_c_hist(2,:));
legend('w0_t', 'w0_r');
title('W0 sinal controle');
xlabel('t');
ylabel('sinal');

% figure();
% hold on;
% grid on;
% plot(t_hist(1,:), theta_eq_hist(1,:)*180/pi);
% legend('theta giro');
% title('theta giro');
% xlabel('t');
% ylabel('angulo');

% figure();
% hold on;
% grid on;
% plot(xd_hist(1,:),xd_hist(2,:));
% plot(pose_frente_trailer_hist(1,:),pose_frente_trailer_hist(2,:)); 
% plot(centro_robo_hist(1,:),centro_robo_hist(2,:)); 
% legend('Desejado','Trailer', 'Frente robo');
% xlabel('X');
% ylabel('Y');

figure();
hold on;
grid on;
plot(xd_hist(1,:),xd_hist(2,:));
plot(pose_final_trailer_hist(1,:),pose_final_trailer_hist(2,:)); 
% plot(centro_robo_hist(1,:),centro_robo_hist(2,:)); 
legend('Desejado','final Trailer', 'Centro robo');
xlabel('X');
ylabel('Y');

% figure();
% hold on;
% grid on;
% plot(xd_hist(1,:),xd_hist(2,:));
% plot(centro_robo_hist(1,:),centro_robo_hist(2,:)); 
% plot(x_atras_hist(1,:),x_atras_hist(2,:)); 
% legend('Desejado', 'Centro robo','atras robo');
% xlabel('X');
% ylabel('Y');

% figure();
% hold on;
% grid on;
% plot(t_hist(1,:), xd_hist(1,:) - centro_trailer_hist(1,:));
% plot(t_hist(1,:), xd_hist(2,:) - centro_trailer_hist(2,:));
% legend('erro trailer x', 'erro trailer y');
% title('erro posições');
% xlabel('t');
% ylabel('sinal');

figure();
hold on;
grid on;
plot(t_hist(1,:), xd_hist(1,:) - pose_final_trailer_hist(1,:));
plot(t_hist(1,:), xd_hist(2,:) - pose_final_trailer_hist(2,:));
legend('erro trailer x', 'erro trailer y');
title('erro posições');
xlabel('t');
ylabel('sinal');
%%
path = [pwd '\ROS\Scripts\DiegoCBA\'];
filename = ['Diego_re_circulo_T' num2str(T) '_Data_' datestr(now,30) '.mat'];
fullname = [path filename];
save(fullname,'t_hist','controlT_hist','w0_c_hist','xd_hist','pose_final_trailer_hist','centro_robo_hist','pose_final_trailer_hist')
