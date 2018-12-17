%% Testar modelo dinâmico do ArDrone em uma trajetória

% Inicialização
close all
clear
clc

try
    fclose(instrfindall);
catch
end

% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_Optitrack')
Arq = fopen(['Exptrajetoria_lemniscata_drone_' NomeArq '.txt'],'w');
cd(PastaAtual)


%% Initialize classes
% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Connect Joystick
J = JoyControl;

% Robot initialization
A = ArDrone;
% detect rigid body ID from optitrack
idA = getID(OPT,A);         % drone ID on optitrack

% gains = [2 3.2 1 3.2 2 15; % Anteriormente 1
%          2 15 2 15 1 5];

% gains = [0.5 0.5 0.5 0.5 4 1.8;
%          1.2 15 1.1 8 1 2.5];
gains = [1 5 1 5 2 15;
    10 3 8 3 1 5];


A.rConnect;
A.rTakeOff;
% A.rGetSensorCalibration;

% %% Window definition
% figure(1)
% axis([-3 3 -3 3 0 3])
% grid on
% A.mCADplot
% drawnow
pause(5)

%% Variables Initialization
a = 1.2;              % "raio" do circulo em x
b = 0.9;              % "raio" do circulo em y
c = 0.3;
w = 0.1;             % angular velocity
% nvoltas = 1;
% tsim = pi*nvoltas/(w*2);

angulos = [];
XX      = [];         % position data
kopt    = 0;          % counter
kdrone  = 0;          % counter

% Time variables
tsim = 50;    % Simulation duration [s]
tout = 100;     % maximum simulation duration [s]
tc = tic;      % drone frequency
tp = tic;      % graph refresh rate
tt = tic;      % trajectory time
t = tic;       % simulation current time
td = tic;
ta = [];
dt = 1/30;

%% Simulation loop
while toc(t) < tsim
    if toc(tc) > 1/30
        ta = [ta toc(tc)];
        tc = tic;
        
        % Trajetória desejada
        % Lemniscata
        
        tt = toc(t);
        A.pPos.Xda = A.pPos.Xd;
        
        A.pPos.Xd(1) = a*sin(2*pi*w*tt) ;            % x
        A.pPos.Xd(7) = a*2*pi*w*cos(2*pi*w*tt);     % dx
        A.pPos.Xd(2) = b*sin(2*pi*2*w*tt);          % y
        A.pPos.Xd(8) = b*2*pi*2*w*cos(2*pi*2*w*tt); % dy
        %         A.pPos.Xd(3) = 1+c*sin(2*pi*w*tt);      % z 
        %         A.pPos.Xd(9) = 2*pi*w*c*cos(2*pi*w*tt);
        A.pPos.Xd(3) = 1.5;                         % z
        A.pPos.Xd(9) = 0;
        
        A.pPos.Xd(6) = atan2(A.pPos.Xd(8),A.pPos.Xd(7));
        if abs(A.pPos.Xd(6) - A.pPos.Xda(6)) > pi
            if A.pPos.Xda(6) < 0
                A.pPos.Xda(6) =  2*pi + A.pPos.Xda(6);
            else
                A.pPos.Xda(6) = -2*pi + A.pPos.Xda(6);
            end
        end
        A.pPos.Xd(12) = (A.pPos.Xd(6) - A.pPos.Xda(6))/dt;
        % ----------------------------------------------------------
        
        %         Get current rigid body information from optitrack
        rb = OPT.RigidBody;
        A = getOptData(rb(idA),A);
        
        A = cUnderActuatedControllerMexido(A,gains);
        A = J.mControl(A);           % joystick command (priority)
        
        % Save data  --------------------------------------------------
        
        % Control signal
        lim = (15*pi/180);
        % U = [phi theta dz dpsi]
        A.pSC.U = [A.pPos.X(4)/lim; A.pPos.X(5)/lim; A.pPos.X(9); A.pPos.X(12)/(100*pi/180)];
        
        % Text file
        fprintf(Arq,'%6.6f\t',[A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' toc(t)]);
        fprintf(Arq,'\n\r');
        
        % Variable
        XX = [XX [A.pPos.Xd; A.pPos.X; A.pSC.Ud; A.pSC.U; toc(t)]];
        
        % ----------------------------------------------------------------
        
        A.rSendControlSignals;
        
    end
    
%         % Draw robot
%         if toc(tp) > 0.2
%             tp = tic;
%             A.mCADplot; hold on
%             plot3(XX(1,:),XX(2,:),XX(3,:),'k-','LineWidth',0.5);
%             plot3(XX(13,:),XX(14,:),XX(15,:),'g-','LineWidth',1);
%             drawnow
%         end
%     
%         % Timeout
%         if toc(t)>tout
%             disp('Time out. Ending simulation...');
%             break
%         end
    
end

% Land drone
if A.pFlag.Connected == 1
    A.rLand;
    %     A.rDisconnect;
end

% % Close txt file
fclose(Arq);

%% Plot results
%
% figure
% plot(ta)


% Trajectory 2D
figure
% axis equal
plot(XX([1,13],:)',XX([2,14],:)')
% axis([-1.5 1.5 -1.5 1.5])
grid on

% Trajectory 3D
figure
% axis equal
plot3(XX([1,13],:)',XX([2,14],:)',XX([3,15],:)')
% axis([-1.5 1.5 -1.5 1.5])
grid on

% Roll and pitch angles
figure
subplot(311),plot(XX(end,:),XX([4 16],:)'*180/pi)
legend('\phi_{Des}[^o]','\phi_{Real}[^o]')
grid
subplot(312),plot(XX(end,:),XX([5 17],:)'*180/pi)
legend('\theta_{Des}[^o]','\theta_{Real}[^o]')
grid
subplot(313),plot(XX(end,:),XX([6 18],:)'*180/pi)
legend('\psi_{Des}[^o]','\psi_{Real}[^o]')
grid

% X and Y
figure
subplot(311),plot(XX(end,:),XX([1 13],:)')
legend('x_{Des}','x_{Real}')
grid
subplot(312),plot(XX(end,:),XX([2 14],:)')
legend('y_{Des}','y_{Real}')
grid
subplot(313),plot(XX(end,:),XX([3 15],:)')
legend('z_{Des}','z_{Real}')
grid

% dX and dY
figure
subplot(311),plot(XX(end,:),XX([7 19],:)')
legend('dx_{Des}','dx_{Real}')
grid
subplot(312),plot(XX(end,:),XX([8 20],:)')
legend('dy_{Des}','dy_{Real}')
grid
subplot(313),plot(XX(end,:),XX([9 21],:)')
legend('dz_{Des}','dz_{Real}')
grid


