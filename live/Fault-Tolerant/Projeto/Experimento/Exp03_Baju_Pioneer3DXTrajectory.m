clear all
close all
clc

try
    fclose(instrfindall);
catch
end

%% Load Class
try
    % Load Classes
    RI = RosInterface;
    setenv('ROS_IP','192.168.0.158')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    
    % Create OptiTrack object and initialize
    OPT = OptiTrack;
    OPT.Initialize;
    
    P = RPioneer(1,'P1');
    % detect rigid body ID from optitrack
    idP = 1;
    
    % Joystick
    J = JoyControl;
    Joy = 1;
    
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
P.pPar.a = 0;
P.pPar.alpha = 0;

rb = OPT.RigidBody;          % read optitrack data
% B = getOptData(rb(idB),B);   % get ardrone data
P = getOptData(rb(idP),P);   % get pioneer data

%% CONTROLLER GAINS
pgains = [0.1 0.1 1];
P.pPar.a = 0;
P.pPar.alpha = 0;

%% TEMPORIZADORES

T_MAX = 15;
T_AMOSTRAGEM = P.pPar.Ts;
W = 2*pi/T_MAX;
rx = 1;
ry = 1;
Xd = [0; 0; 0];

T = tic;
Ta = tic;
D = tic;

P.rEnableMotors;

while true %toc(T) < T_MAX
if toc(Ta) > T_AMOSTRAGEM 
    Ta = tic;
    XdA = Xd;
%    Lemniscata (8')
%     Xd = [rx*sin(2*W*toc(T)); ry*sin(W*toc(T)); 0];

%    Circunferencia
%     Xd = [rx*cos(W*toc(T)); ry*sin(W*toc(T)); 0];
    
%    Posição
    Xd = [0; 0; 0];
    
    dXd = (Xd-XdA)/toc(D);
    D = tic;
    
    % Acquire sensors data        
    % Get optitrack data        
    rb = OPT.RigidBody;             % read optitrack
    
    % Pioneer
    P = getOptData(rb(idP),P);
    
    P.pPos.Xd(1:3) = Xd;
    P.pPos.Xd(7:9) = dXd;
    
    P = fKinematicControllerExtended(P,pgains);        % new controller (by timotiu 2020)
    
    P = J.mControl(P);
    
    P.rCommand;
    
    if J.pFlag == 1
       break 
    end
    
end
end

P.rDisableMotors;



















