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
    
    nPioneer = 1;
    
     P = RPioneer(1,'P1');
     
    
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
dados=[];
Rastro.Xd = [];
Rastro.X = [];
P.pPar.a = 0;
P.pPar.alpha = 0;

% rb = OPT.RigidBody;          % read optitrack data
% B = getOptData(rb(idB),B);   % get ardrone data
% P = getOptData(rb(idP),P);   % get pioneer data

%% CONTROLLER GAINS
P.pPar.a = 0;
P.pPar.alpha = 0;

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


P.rEnableMotors;                        %Ativa Motor Pioneer

while toc(ta) < tmax && Joy == 0
if toc(ta) > tmax 
    ta = tic;
    
    % Acquire sensors data        
    % Get optitrack data        
%     rb = OPT.RigidBody;             % read optitrack
    
    % Pioneer
%     P = getOptData(rb(idP),P);
    
%     P.pPos.Xd(1:3) = Xd;
%     P.pPos.Xd(7:9) = dXd;
    
    P = fKinematicControllerExtended(P,pgains);        % new controller (by timotiu 2020)
    
            % Armazenar dados da simulação
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];
        Rastro.Xd = [Rastro.Xd; P.pPos.Xd(1:2)'];  % formação desejada
        Rastro.X  = [Rastro.X; P.pPos.X(1:2)'];    % formação real
        
        % Enviar sinais de controle para o robô
        % P.rSendControlSignals;     
        
          %% Desenha o robô
            if toc(tp) > 0.1
                tp = tic;
                
                P.mCADdel
                P.mCADplot2D('r')   % Visualização 2D
         
                hold on
                plot(Rastro.Xd(:,1),Rastro.Xd(:,2),'r-');
                hold on;
                plot(Rastro.X(:,1),Rastro.X(:,2),'black');
                
                
                drawnow
            end
    end
end

P.pSC.Ud([1 2]) = 0;

    P = J.mControl(P);
    
    P.rCommand;
    
    
    
    if J.pFlag == 1
       break 
    end
    
end
end

while J.pFlag == 0 && Joy == 1
if toc(ta) > tmax 
    ta = tic;
    % Acquire sensors data        
    % Get optitrack data        
%     rb = OPT.RigidBody;             % read optitrack
%     
%     % Pioneer
%     P = getOptData(rb(idP),P);
    P = J.mControl(P);
    
    P.rCommand;
    P.pSC.Ud = [0; 0];
end
end

P.rDisableMotors;








