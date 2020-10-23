clear
close all
clc

try
    fclose(instrfindall);
catch
    
end

%% Rotina para buscar pasta raiz
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA ELT334';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))

%% Definindo o Robï¿½
P = Pioneer3DX;
PJ = Pioneer3DX(2);
P.rConnect; % Descomentar para realiazaï¿½ï¿½o de experimento
P.rSetPose([0 0 0 0]);

% Tempo de esperar para inï¿½cio do experimento/simulaï¿½ï¿½o
pause(1)



%%


J = JoyControl;

% Setup variaveis de ambiente Ros Master, Ros Matlab

setenv('ROS_IP','192.168.0.158')                        %Ip do desktop
setenv('ROS_HOSTNAME','192.168.0.158')                  %Host name = ip
setenv('ROS_MASTER_URI','http://192.168.0.148:11311/')  %Ip do master (raspberry)
rosinit                                                 %iniciar a comunicação


[pub,cmd_vel] = rospublisher('/RosAria/cmd_vel','geometry_msgs/Twist'); %Publicador
[pub2,cmd_vel2] = rospublisher('/matlab/cmd_vel','geometry_msgs/Twist');

cmd_vel.Linear.X = 0;
cmd_vel.Angular.Z = 0;
send(pub,cmd_vel);          %Enviar sinais

cmd_vel2.Linear.X = 0;
cmd_vel2.Angular.Z = 0;
send(pub2,cmd_vel2);          %Enviar sinais

sub = rossubscriber('/RosAria/pose','nav_msgs/Odometry');
sub2 = rossubscriber('/matlab/pose','nav_msgs/Odometry');

% sub = rossubscriber('/matlab/cmd_vel','geometry_msgs/Twist');

%% Definindo a Figura que irï¿½ rodar a simulaï¿½ï¿½o
figure(1)
hold on
grid on
title('Simulação');
axis([-3 3 -3 3])


% xd = input('Entre com posição final no eixo x (xd) = ');
% yd = input('Entre com posição final no eixo y (yd) = ');
% od = input('entre com posiï¿½ï¿½o orientaï¿½ï¿½o em graus =');
% if od > 180
%     od = od - 360;
% end
%% Tempo Real
a=0;
tmax = 60;
ta = tic;
t = tic;
U=[];
tp=tic;
tc=tic;
Dados=[]
pose2 = sub2.LatestMessage;

XB = pose2.Pose.Pose.Position.X;
YB = pose2.Pose.Pose.Position.Y;
theta = pose2.Pose.Pose.Orientation.Z;
%% Inicio da simulaï¿½ï¿½o
while (toc(t) <= tmax)
    % Sub-rotina de controle
    PJ = J.mControl(PJ);
    
    cmd_vel2.Linear.X = PJ.pSC.Ud(1);
    cmd_vel2.Angular.Z = PJ.pSC.Ud(2);
    send(pub2,cmd_vel2)
    if toc(tc) > 0.1
        a=0;
        % Inicio da realimentaï¿½ï¿½o
        tc = tic;
        XBA=XB ;
        YBA=YB;
        thetaA=theta;
        % Pegando os dados da bolinha
        pose2 = sub2.LatestMessage;
        
        XB = pose2.Pose.Pose.Position.X;
        YB = pose2.Pose.Pose.Position.Y;
        theta = pose2.Pose.Pose.Orientation.Z;
        V= [(XB-XBA)/0.1 (YB-YBA)/0.1 (theta-thetaA)/0.1]';
        % Pegando os dados do robo
        pose = sub.LatestMessage;
        
        P.pPos.Xc(1) = pose.Pose.Pose.Position.X;
        P.pPos.Xc(2) = pose.Pose.Pose.Position.Y;
        P.pPos.Xc(6) = quat2eul(pose.Pose.Pose.Orientation.readQuaternion);
        %       P.rGetSensorData;
        %syms psie Ro
        x = P.pPos.Xc(1);          % em [m]
        y = P.pPos.Xc(2);          % em [m]
        psi = P.pPos.Xc(6); % em [ï¿½]
        v=0;
        Ro=((XB-x)^2+(YB-y)^2)^(1/2)
        alfa= atan2((XB-y),(YB-x))-psi;
        K1=0.6/(0.1+abs(XB-x));
        K2=0.6/(0.1+abs(YB-y));%Ro/(1+abs(Ro));
        K2=.65;
        C = [cos(psi) sin(psi) -a*sin(alfa) ; -sin(psi) cos(psi) a*cos(alfa) ; 0 0 1]; %inversa da matriz de controle
        Vx=((XB-XBA)/0.1)+k1*(XB-x);
        Vy=((YB-YBA)/0.1)+k2*(YB-y);
        Vw=u(2,1);
        U=[u(1,1) v u(2,1)];
        U=C*[Vx ; Vy ; Vw];
        v=0
        if abs(alfa)>pi
            if alfa>0
                alfa=-2*pi+alfa
            else
                alfa=2*pi+alfa
            end
        end
        if Ro<1.0
            a=0.5;                  %             u(1,1) = 0;
             %             if abs(psie)<0.01
            %                 break
            %             end
            %         else
%             u(1,1)=K1*(Ro/(exp(Ro^2)*cos(psie)));
%             u(2,1)=((u(1,1)*tan(psie))/(Ro))+((K1*psie)/exp(psie^2));
            %         u = K2*(atan(Ro))
            %         w = (u*sin(alfa))/Ro + (atan(alfa))*((alfa^2)+1)/2
%             disp(toc(t))
            %end
            while
                if Ro<1.5
                    a=0.5;
                else
                    a=0;
                end
            end
        end
        P.pSC.Ud = ([u; w]);
        cmd_vel.Linear.X = P.pSC.Ud(1);
        cmd_vel.Angular.Z = P.pSC.Ud(2);
        send(pub,cmd_vel)
        %         P.rSendControlSignals;
        Dados=[Dados;[x y psi Ro alfa u w toc(t)]];
        if toc(tp) > 0.1
            P.mCADdel
            P.mCADplot2D('r')   % Visualização 2D
            drawnow
        end
    end
end

P.pSC.Ud([1 2]) = 0;
P.rSendControlSignals;