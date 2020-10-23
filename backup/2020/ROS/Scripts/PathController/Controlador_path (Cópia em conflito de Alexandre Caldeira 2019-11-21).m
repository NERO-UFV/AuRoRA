%% 3D Line Formation Pioneer-Drone
% Pioneer is the reference of the formation
% The formation variables are:
% Q = [ xf yf zf rho alfa beta ]
% Initial Comands

clear; close all; clc;
try
    fclose(instrfindall);
catch
end

% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Classes

%% Load Class
try
    % Load Classes
    RI = RosInterface;
    setenv('ROS_IP','')
    setenv('ROS_HOSTNAME','192.168.0.100')
    setenv('ROS_MASTER_URI','http://192.168.0.108:11311')
    RI.rConnect('192.168.0.108');
    P = RPioneer(1);
    
    % Joystick
    %     J = JoyControl;
    
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
% nLandMsg = 3;
% btnEmergencia = 0;
% ButtonHandle = uicontrol('Style', 'PushButton', ...
%                          'String', 'land', ...
%                          'Callback', 'btnEmergencia=1', ...
%                          'Position', [50 50 400 300]);


%% Variable initialization
data = [];
P.rGetSensorData;

% Time variables initialization
T_CONTROL = 0.1; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 60;

T = 10;
w = 2*pi/T;

fprintf('\nStart..............\n\n');

% Parametros dos Controladores
model = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295 ];
%          X     Y     Z    Psi
gains = [  1     1     1     1 ...
    1.5   1.5   3     1 ...
    1     1     1     1 ...
    1     1     1     1];

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;
a = 0.10;
K1 = diag([1 1]);
K2 = diag([.2 .3]);
dist = 1000;
tol = 0.1;

%Caminho
Ve = 0.1;
inc = 1;
term = 150;
s = 0:inc:term;
x = 3*cos(pi*s/100) + 1;
y = 0.5*sin(pi*s/100);

C = [x; y];
joy = vrjoystick(1);
dist_final = 1000;
tol_final = 0.1;

try
%     while toc(t) < T_MAX
        while dist_final > tol_final
        
        if toc(t_control) > T_CONTROL
            
            P.pPar.Ts = toc(t_incB);
            t_incB = tic;
            t_control = tic;
            
            a_cont = axis(joy);
            Vd = -Ve*a_cont(5);
            X = P.pPos.X;
            dist_final = norm(C(:,end)-X(1:2));
            
            [dist, ind] = calcula_ponto_proximo(C,X(1:2));
            tan_inc(1,1) = 0.5*pi/100*sin(pi*s(ind)/100);
            tan_inc(2,1) = 0.5*pi/100*cos(pi*s(ind)/100);
            vtan = Vd*tan_inc/norm(tan_inc);
            
            % Trajectory Planner
            %             Xd = [0.5*cos(w*toc(t)) 0.5*sin(w*toc(t))]';
            %             dXd = [-w*0.5*sin(w*toc(t)) w*0.5*cos(w*toc(t))]';
            
            Xd = [C(:,ind)];
            
            if dist > tol
                %             dXd1 = [vector_vel; 0];
                dXd = [0 0]';
            else
                dXd = [vtan];
            end
            
            Xtil = Xd - X(1:2);
            
            X(1:2)
            F = [cos(X(6)) -a*sin(X(6));
                sin(X(6))  a*cos(X(6))];
            
            Xc = dXd + K1*tanh(K2*Xtil);
            
            P.pSC.Ud = F\Xc;
            %             F\Xc
            
            % Ardrone
            P.rGetSensorData;
            
            % Joystick Command Priority
            %             P = J.mControl(P);                    % joystick command (priority)
            P.rCommand;
        end
    end
catch ME
    disp('Bebop Landing through Try/Catch Loop Command');
end


% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

