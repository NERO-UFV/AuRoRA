%% CONTROLE DE TRAJETÓRIA  UTILIZANDO OPTITRACK
% Controle independente de trajetórias com pioneer e drone
%  Este algoritmo deve rodar no pc do optitrack, que fará todo
%  processamento e enviará para rede os sinais de controle para o pioneer.
% *Este pc precisará receber os dados dos sensores de velocidade do pioneer


clear
close all
clc
% Fecha todas possíveis conexões abertas
try
    fclose(instrfindall);
catch
end

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Definição da janela do gráfico
% fig = figure(1);
% axis([-5 5 -5 5])
% axis equal
%
%% Classes initialization
% Robot
P = Pioneer3DX;
A = ArDrone;

% Network
Rede = NetDataShare;

% OptiTrack
OPT = OptiTrack;
OPT.Initialize;

% Joystick control
J = JoyControl;

%% Robot connection/initialization
A.rConnect;
A.rTakeOff;
% A.rGetSensorCalibration;
pause(8);

%% Define robot initial position
%  Get current rigid body information from optitrack
  rb = OPT.RigidBody;
    if rb(1).isTracked
        P = getOptData(rb(1),P);
    end
    if rb(2).isTracked
        A = getOptData(rb(2),A);
    end
P.rSetPose(P.pPos.X([1 2 3 6]));       % define Pioneer initial pose

%% Network communication check
tm = tic;
while true
    
    if isempty(Rede.pMSG.getFrom)
        Rede.mSendMsg(P);
        if toc(tm) > 0.1
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        end
    elseif length(Rede.pMSG.getFrom) > 1
        if isempty(Rede.pMSG.getFrom{2})
            Rede.mSendMsg(P);
            
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
            
        else
            break
        end
    end
end
clc
disp('Data received. Segue o jogo...');

%% Variables
% Xa = P.pPos.X(1:6);    % postura anterior
data = [];
Rastro.Xd = [];
Rastro.X = [];
Rastro.O = [];
Rastro.U  = [];
Rastro.Ud = [];
XA = [];
k = 0;
X=[];
% Controller gains
% K1 = diag([0.5 0.01]);    % limitador do sinal
% K2 = diag([1 1]);

K1 = diag([0.8 0.01]);      % limitador do sinal
K2 = diag([1 .5]);

%% Trajectory variables
a  = 1.2;                 % x distance
b  = 0.8;                 % y distance
c  = 1;                   % heigth
wp = 0.1;                 % angular velocity for pioneer
wa = 0.1;                 % angular velocity for ardrone

% nvoltas = 1;              % number of complete "rounds"
% tsim = 2*pi*nvoltas/wp;   % simulation time
tsim = 60;                % simulation time
%% Simulation

% Temporização
tap = 0.1;     % taxa de atualização do pioneer
dt = 1/30;
% timeout = 200;   % tempo máximo de duração da simulação
t = tic;
tc = tic;
tp = tic;

while toc(t) < tsim
    
    if toc(tc) > 1/30
        
        tc = tic;
        
        %% Data aquisition
        % Get network data
        Rede.mReceiveMsg;
        % If there is data from optitrack, overwrite position values
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{2}(29:30); % current velocities (robot sensors)
            X  = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
            %             disp('Velocity received!');
        else
            k = k+1;
            disp('No data received from robot......')
        end
        
        rb = OPT.RigidBody;
        n = length(rb);
        if ~isempty(n)
            for ii = 1:n
                switch rb(ii).Name(1)
                    
                    % Pioneer
                    case 'P'
                        P = getOptData(rb(ii),P);

                      P.pPos.X(7:9) = (P.pPos.X(1:3)-P.pPos.Xa(1:3))/dt;
                        P.pPos.X(10:11) = (P.pPos.X(4:5)-P.pPos.Xa(4:5))/dt;
                        if abs(P.pPos.X(6) - P.pPos.Xa(6)) > pi
                            if P.pPos.Xa(6) < 0
                                P.pPos.Xa(6) =  2*pi + P.pPos.Xa(6);
                            else
                                P.pPos.Xa(6) = -2*pi + P.pPos.Xa(6);
                            end
                        end
                        P.pPos.X(12) = (P.pPos.X(6)-P.pPos.Xa(6))/dt;
                        
                        P.pPos.Xa = P.pPos.X;
                        
                        % ArDrone
                    case 'A'
                        A = getOptData(rb(ii),A);
                    otherwise
                        disp('Unknown rigid body type. (Known types: "A" or "P")');
                end
            end
            
        else
            disp('No rigid body tracked');
        end
        
        A.rGetAngles;      % get roll/pitch angles from drone sensors
        
        %% Trajectory
        % Pioneer /////////////////////////////////////////////////
        % infinito (8')
        ta = toc(t);
        P.pPos.Xd(1)  = a*sin(wp*ta);       % posição x
        P.pPos.Xd(2)  = b*sin(2*wp*ta);     % posição y
        P.pPos.Xd(7)  = a*wp*cos(wp*ta);     % velocidade em x
        P.pPos.Xd(8)  = 2*b*wp*cos(2*wp*ta); % velocidade em y

        % ArDrone ////////////////////////////////////////////////
        tt = toc(t);
        A.pPos.Xda = A.pPos.Xd;
        
        A.pPos.Xd(1) = a*sin(wa*tt) ;                    % x
        A.pPos.Xd(2) = b*sin(2*wa*tt);                   % y
        A.pPos.Xd(3) = c;                                % z
        A.pPos.Xd(7) = a*wa*cos(wa*tt);                  % dx
        A.pPos.Xd(8) = b*2*wa*cos(2*wa*tt);              % dy
        A.pPos.Xd(9) = 0;                                % dz
        
        A.pPos.Xd(6) = atan2(A.pPos.Xd(8),A.pPos.Xd(7)); % dPsi
        % Angle adjust
        if abs(A.pPos.Xd(6) - A.pPos.Xda(6)) > pi
            if A.pPos.Xda(6) < 0
                A.pPos.Xda(6) =  2*pi + A.pPos.Xda(6);
            else
                A.pPos.Xda(6) = -2*pi + A.pPos.Xda(6);
            end
        end
        A.pPos.Xd(12) = (A.pPos.Xd(6) - A.pPos.Xda(6))/dt;
        
        %% Control
        % Pioneer /////////////////////////////////////////////////

        P.fControladorDinamico(P);
        
        Rede.mSendMsg(P);  % send data to network
        
        % ArDrone /////////////////////////////////////////////////
        A = cUnderActuatedController(A);
        A = J.mControl(A);           % joystick command (priority)
        A.rSendControlSignals;
        
        %% Data saving for plot
        
        % Pioneer ////////////////////////////////////////////////
        % save data to plot
        Rastro.Xd = [Rastro.Xd; P.pPos.Xd(1:2)'];  % desired trajectory
        Rastro.O  = [Rastro.O; P.pPos.X(1:2)'];    % optitrack position
        Rastro.X  = [Rastro.X; X'];                % robot sensors position
        Rastro.Ud = [Rastro.Ud; P.pSC.Ud'];        % control signal
        Rastro.U = [Rastro.U; P.pSC.U'];           % current velocities
        
        % ArDrone /////////////////////////////////////////////////
        
        % Control signal
        lim = (15*pi/180);
        % U = [phi theta dz dpsi]
        A.pSC.U = [A.pPos.X(4)/lim; A.pPos.X(5)/lim; A.pPos.X(9); A.pPos.X(12)/(100*pi/180)];
        
        % Variable
        XA = [XA [A.pPos.Xd; A.pPos.X; A.pSC.Ud; A.pSC.U; toc(t)]];
        
        %% Draw robots
        %         if toc(tp) > 1/30
        %             tp = tic;
        %             P.mCADdel
        %             delete(fig);
        %
        %             P.mCADplot(1,'k')
        %             A.mCADplot;
        %             hold on
        %             axis([-5 5 -5 5])
        %             grid on
        %             drawnow
        %         end
        
    end
    
end

%%  Stop/Land robot
% Pioneer /////////////////////////////////////////////

% define control signal null
P.pSC.Ud = [0 ; 0];
% Send to network (a few times to be sure)
for ii = 1:5
    Rede.mSendMsg(P);
end

% ArDrone ///////////////////////////////////////////
% Land drone
if A.pFlag.Connected == 1
    A.rLand;
end


%% Results
% Pioneer /////////////////////////////////////////////////////
% Control Signals
figure;
subplot(211)
plot(Rastro.Ud(:,1)),hold on;
plot(Rastro.U(:,1)), legend('Ud Linear ','Real')
grid on
subplot(212)
plot(Rastro.Ud(:,2)),hold on
plot(Rastro.U(:,2)), legend('Ud Angular','Real')
grid on

% Trajectory
figure;
plot(Rastro.Xd(:,1),Rastro.Xd(:,2),'LineWidth',1.5),hold on
plot(Rastro.X(:,1),Rastro.X(:,2),'--')
plot(Rastro.O(:,1),Rastro.O(:,2),'-')
legend('Desejado','Sensor','Optitrack')
grid on

% ArDrone ////////////////////////////////////////////////////////
% Roll and pitch angles
figure
subplot(311),plot(XA(end,:),XA([4 16],:)'*180/pi)
legend('\phi_{Des}[^o]','\phi_{Atu}[^o]')
grid
subplot(312),plot(XA(end,:),XA([5 17],:)'*180/pi)
legend('\theta_{Des}[^o]','\theta_{Atu}[^o]')
grid
subplot(313),plot(XA(end,:),XA([6 18],:)'*180/pi)
legend('\psi_{Des}[^o]','\psi_{Atu}[^o]')
grid

% Trajectory 2D
figure
% axis equal
plot(XA([1,13],:)',XA([2,14],:)')
% axis([-1.5 1.5 -1.5 1.5])
grid on

% Trajectory 3D
figure
% axis equal
plot3(XA([1,13],:)',XA([2,14],:)',XA([3,15],:)')
% axis([-1.5 1.5 -1.5 1.5])
grid on


% X and Y
figure
subplot(311),plot(XA(end,:),XA([1 13],:)')
legend('x_{Des}','x_{Atu}')
grid
subplot(312),plot(XA(end,:),XA([2 14],:)')
legend('y_{Des}','y_{Atu}')
grid
subplot(313),plot(XA(end,:),XA([3 15],:)')
legend('z_{Des}','dz_{Atu}')
grid

% % dX and dY
% figure
% subplot(311),plot(XX(end,:),XX([7 19],:)')
% legend('dx_{Des}','dx_{Atu}')
% grid
% subplot(312),plot(XX(end,:),XX([8 20],:)')
% legend('dy_{Des}','dy_{Atu}')
% grid
% subplot(313),plot(XX(end,:),XX([9 21],:)')
% legend('dz_{Des}','dz_{Atu}')
% grid


% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
