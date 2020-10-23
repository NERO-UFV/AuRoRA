% Limpeza de variaveís e códigos
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
    setenv('ROS_IP','192.168.0.163')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    
% Create OptiTrack object and initialize
%     OPT = OptiTrack;
%     OPT.Initialize;
    
        
    P=RPioneer(1,'P1');    
    
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


% Definindo os parametros do pioneer
P.pPar.a = 0;
P.pPar.alpha = 0;
pgains = [.1 .1 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Variáveis para o caminho:
%Definindo variaveis:
xs  = 0;                        ys = 0;               %Posição inicial
xf  = 1;                        yf = 2;               %Posição final
ths = 0;                        thf = (120)*(pi/180);   %Orientação inicial/final

%Pontos de controle:
b0 = [xs;ys];                   b5 = [xf;yf];   
c0 = 1;                         c5 = 1;
d0 = c0.*[cos(ths),sin(ths)];   d5 = c5.*[cos(thf),sin(thf)];

b1 = b0 + [(1/5).*d0]';         b4 = b5 - [(1/5).*d5]';

b2 = b1 + [(1/5).*d0]';         b3 = b4 - [(1/5).*d5]';        %Inicialização aleatória


ControlPoints = [b0,b1,b2,b3,b4,b5]; %=> N = 6;

%Plot da curva
t = 0:0.01:1;
N = 6;
Sum = 0;    

for i=0:N-1
    Sum = Sum + nchoosek(N-1,i)*(1-t).^(N-1-i).*(t.^i).*ControlPoints(:,i+1);
end
Curve = Sum;

plot(xs,ys,'k*')
hold on
plot(xf,yf,'r*')
hold on
plot(ControlPoints(1,:),ControlPoints(2,:),'bo')
hold on
plot(ControlPoints(1,:),ControlPoints(2,:),'k*')
hold on
plot(Curve(1,:),Curve(2,:),'b--')
hold on
plot(Curve(1,:),Curve(2,:),'b*')
axis([-1 4 -1 4])
grid on

        
%% Rotina da simulação:
t=tic;  ta=tic;   tp = tic;         tc=tic; tcmax=35;
        tmax=40;
k1=0.6; k2=0.4;   it = 0;

while toc(t)<tmax
    if toc(ta)>0.1
        ta=tic;
        
        P.pPos.Xda     = P.pPos.Xd;
        it=it+1;
        
       
        time = toc(tc)/(tcmax);
        if toc(tc)>tcmax
            time=1;
        end
        %Pegar informação da posição e velocidade real do robô
        P.rGetSensorData;
        
        %% Cálculo da curva:
        N = 6;
        Sum = 0;

        for i=0:N-1
            Sum = Sum + nchoosek(N-1,i)*(1-time).^(N-1-i).*(time.^i).*ControlPoints(:,i+1);
        end
        Curve = Sum;
    end
 end
        
        %% Robot control
        %Posição do ponto desejado:
        P.pPos.Xd(1)= Curve(1);
        P.pPos.Xd(2)= Curve(2);
        
        % Pegando os dados do robo
%         disp('.........................')
%         disp(P.pPos.X(1))
        
        P.rGetSensorData;
%         disp(P.pPos.X(1))
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
%%

P.rEnableMotors; %Ativa motor do pioneer


% Laço de simulação
while toc(t) < tmax
    % Laço de controle
    if toc(ta) > tcmax
        ta = tic;
        
        XA = P.pPos.X;
        % Pegar sinais de posição do pioneer
        P.rGetSensorData;

        % Controle
        P = fKinematicControllerExtended(P,pgains);

        P = J.mControl(P);
        
        % Enviar sinais de controle
        P.rCommand;
        P.pSC.Ud = [0; 0];
        if J.pFlag == 1
            break 
        end
    
    end
    % Laço de plot
    if toc(Tp) > T_PLOT
        try
            P.mCADdel;
        catch
        end
        
        P.mCADplot(1,'k');
        Traj = plot([XdA(1) Xd(1)],[XdA(2) Xd(2)],'--k','LineWidth',1.6);
        Rastro = plot([XA(1) P.pPos.X(1)],[XA(2) P.pPos.X(2)],'r','LineWidth',1.6);
        drawnow
    end
end
P.rDisableMotors;


