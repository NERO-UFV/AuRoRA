clear all
% Criando uma variável para representar o Robô
P = Pioneer3DX;
J = JoyControl;

% Setup variaveis de ambiente Ros Master, Ros Matlab

setenv('ROS_IP','192.168.0.158')                        %Ip do desktop
setenv('ROS_HOSTNAME','192.168.0.158')                  %Host name = ip
setenv('ROS_MASTER_URI','http://192.168.0.148:11311/')  %Ip do master (raspberry)
%  rosinit;                                                %iniciar a comunicação


[pub,cmd_vel] = rospublisher('/RosAria/cmd_vel','geometry_msgs/Twist'); %Publicador
 [pub2,cmd_vel2] = rospublisher('/matlab/cmd_vel','geometry_msgs/Twist'); 

 cmd_vel2.Linear.X = 0;
 cmd_vel2.Angular.Z = 0;
 send(pub2,cmd_vel2);          %Enviar sinais

% sub = rossubscriber('/scan','sensor_msgs/LaserScan');                  %Laser scan

sub = rossubscriber('/RosAria/pose','nav_msgs/Odometry');
% sub2 = rossubscriber('/matlab/pose','nav_msgs/Odometry');

% sub = rossubscriber('/matlab/cmd_vel','geometry_msgs/Twist');

pose = sub.LatestMessage;

% pause(5);

%X = pose.Pose.Pose.Position.X;
%Y = pose.Pose.Pose.Position.Y;

%VerdadeiroTheta = quat2eul(pose.Pose.Pose.Orientation.readQuaternion)*180/pi;

Dados=[];%teste dos testes

cmd_vel.Linear.X = 0;       %Velocidade linear
cmd_vel.Angular.Z = 0.0;    %Velocidade angular
send(pub,cmd_vel);          %Enviar sinais

%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nLandMsg = 3;
% btnEmergencia = 0;
% P.pFlag.EmergencyStop = 0;
% ButtonHandle = uicontrol('Style', 'PushButton', ...
%                          'String', 'land', ...
%                          'Callback', 'btnEmergencia=1', ...
%                          'Position', [50 50 400 300]);


while true
    P = J.mControl(P);
    
    cmd_vel.Linear.X = P.pSC.Ud(1);
    cmd_vel.Angular.Z = P.pSC.Ud(2);
  send(pub,cmd_vel)
end
%% EMERGÊNCIA
        drawnow
        if btnEmergencia == 1
            P.pFlag.EmergencyStop = 1;
        end
        if btnEmergencia ~= 0 || P.pFlag.EmergencyStop ~= 0 
            disp('Pioneer stopping by  ');

            % Send 3 times Commands 1 second delay to Drone Land
            for i=1:nLandMsg
                P.pSC.Ud = [0 0]';
                cmd_vel.Linear.X = P.pSC.Ud(1);
                cmd_vel.Angular.Z = P.pSC.Ud(2);
                send(pub,cmd_vel)

            end
            break;
        end
        
        pose = sub.LatestMessage;
        
        X = pose.Pose.Pose.Position.X;
        Y = pose.Pose.Pose.Position.Y;
        
        %Theta = quat2eul(pose.Pose.Pose.Orientation.readQuaternion)*180/pi;
         Dados=[Dados;[X Y ];%Theta]];
             end

%% EMERGÊNCIA FORA DO LAÇO    
    for i=1:nLandMsg
        P.pSC.Ud = [0 0]';
        cmd_vel.Linear.X = P.pSC.Ud(1);
        cmd_vel.Angular.Z = P.pSC.Ud(2);
        send(pub,cmd_vel)

    end
    
    
%  rosshutdown