close all
clear all
clc

try
    fclose(instrfindall);
catch
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              Load Class                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

try
    % Load Classes
    
    RI = RosInterface;
    setenv('ROS_IP','192.168.0.158')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    
%     % Inicializando o OptiTrack
    OPT = OptiTrack;    % Criando o OptiTrack
    OPT.Initialize;     % Iniciando o OptiTrack
    
    P{1} = RPioneer(1,'P1'); % Pìoneer3DX Experimento
    P{1}.rDisableMotors;
    
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


subLaser = rossubscriber('/scan','sensor_msgs/LaserScan');
Laser = subLaser.LatestMessage;
H = Laser.plot;
axis equal
axis([-2 6 -6 4])
hold on

while true
    Laser = subLaser.LatestMessage;
    try
        delete(H)
    catch
    end
    
    H = Laser.plot;
    H.Color = 'b';
    axis equal
    axis([-2 20 -20 20])
    drawnow
end