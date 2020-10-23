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
    P = RPioneer(1,'RosAria',0);
    
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
T_MAX = 40;


fprintf('\nStart..............\n\n');

t_int = tic;
t  = tic;
t_control = tic;

% rX = 2;           % [m]
% rY = 2;           % [m]
% T = 60;             % [s]
% Tf = T*2;            % [s]
% w = 2*pi/T;         % [rad/s]
% ww = 1*w;
% phase = 0;
% cont = 0;

r = 1.5;
T = 60;

pgains = [0.35 0.35 0.8 0.8];
Kp1 = diag([pgains(1), pgains(2)]);
Kp2 = diag([pgains(3), pgains(4)]);

kx = 1.5;
ky = 1.5;
kt = 1.5;

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
%             P.pPos.Xd(6) = atan2(P.pPos.Xd(2),P.pPos.Xd(1));
%             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual);-ww*rY*sin(ww*t_atual)];
%             P.pPos.Xd(12) = P.pPos.Xd(2,:)./(P.pPos.Xd(2,:).^2+P.pPos.Xd(1,:).^2);
            

            vd = 2*pi*r/T;
            wd = vd/r;
            
            P.pPos.Xd(7) = vd*cos(P.pPos.Xd(6));
            P.pPos.Xd(8) = vd*sin(P.pPos.Xd(6));
            P.pPos.Xd(12) = wd;
            
            P.pPos.Xd(1) = P.pPos.Xd(1) + P.pPos.Xd(7)*toc(t_int);
            P.pPos.Xd(2) = P.pPos.Xd(2) + P.pPos.Xd(8)*toc(t_int);
            P.pPos.Xd(6) = P.pPos.Xd(6) + P.pPos.Xd(12)*toc(t_int);
            t_int = tic;

%             P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual + phase);-rY*cos(ww*t_atual + phase)+rY];
%             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual + phase);ww*rY*sin(ww*t_atual + phase)];

            
            
%           K = [ cos(P.pPos.X(6)), -P.pPar.a*sin(P.pPos.X(6)); ...
%                 sin(P.pPos.X(6)), +P.pPar.a*cos(P.pPos.X(6))];
%             
%           P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
%             
%           P.pSC.Ud(1:2) = K\(P.pPos.Xd(7:8) + Kp1*tanh(Kp2*P.pPos.Xtil(1:2)));
          P.pPos.Xtil = P.pPos.Xd - P.pPos.Xc;
          K = [ cos(P.pPos.Xc(6)) sin(P.pPos.Xc(6))    0; ...
                -sin(P.pPos.Xc(6)) cos(P.pPos.Xc(6))   0; ...
                        0                   0           1];
                    
          XtilCor = K*P.pPos.Xtil([1 2 6]);
          
          P.pSC.Ud(1) = vd*cos(XtilCor(3)) + kx*XtilCor(1);
          P.pSC.Ud(2) = wd + vd*(ky*XtilCor(2) + kt*sin(XtilCor(3)));
          
% %           P.pSC.Ud(1) = vd;
% %           P.pSC.Ud(2) = wd;
          P.pSC.Ud
          
          P = J.mControl(P);                    % joystick command (priority)


%             B.pSC.Ud
          P.rCommand;
          
          xd_hist = [xd_hist P.pPos.Xd(1:2)];
          x_hist = [x_hist P.pPos.X(1:2)];
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
title('XY');
xlabel('X [m]');
ylabel('Y [m]');

figure();
hold on;
grid on;
plot(t_hist(1,:),xd_hist(1,:) - x_hist(1,:));
plot(t_hist(1,:),xd_hist(2,:) - x_hist(2,:));
title('erro');
xlabel('tempo');
axis([0 T_MAX -1 1])
ylabel('erro [m]');

