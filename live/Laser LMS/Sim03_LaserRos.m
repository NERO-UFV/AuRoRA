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
X = Laser.readCartesian;
dXp = X(1:end-1,:);

figure
H = plot(-X(:,2),X(:,1),'.b');
hold on
grid on
axis equal
axis([-20 20 -1 20])
for ii = 2:size(X,1)
   dXp(ii-1,:) = X(ii,:) - X(ii-1,:); 
end
[~,M] = max(dXp(:,1));
[~,m] = min(dXp(:,1));

while true
    Laser = subLaser.LatestMessage;    
    X = Laser.readCartesian;
    
    try
        delete(H)
    catch
    end
    
    H = plot(-X(:,2),X(:,1),'.b');
    
    drawnow
end











