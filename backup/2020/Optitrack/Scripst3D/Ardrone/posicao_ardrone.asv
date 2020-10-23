%% Testar modelo dinâmico do ArDrone em controle de posição
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
Arq = fopen(['drone_position_' NomeArq '.txt'],'w');
Dronevar = fopen('drone_var','aa');

cd(PastaAtual)



%% Initialize classes
% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Connect Joystick
J = JoyControl;

% Robot initialization
A = ArDrone;
A.rConnect;
% A.rTakeOff;

%% Robot initial pose
idA = getID(OPT,A);           % drone ID on optitrack
rb  = OPT.RigidBody;          % read optitrack data again
A   = getOptData(rb(idA),A);  % populate robot pose variables


%% Window definition
figure(1)
axis([-3 3 -3 3 0 3])
grid on
A.mCADplot
drawnow
pause(2)

while true
    
    rb      = OPT.RigidBody;
    A       = getOptData(rb(idA),A);
    
    
    A.mCADplot
    drawnow
    
    A = J.mControl(A);           % joystick command (priority)
    pause(.1)
end



break;
%% Variables Initialization
% Desired positions [x y z psi]
% Xd = [ 0    0     1.5    0;
%        .75   .75  0.75  -pi/2;
%        .75  -.75  1      pi/2;
%       0     0     1.5    pi;
%       -.75  -.75  0.75   pi/2;
%       -.75   .75  1     -pi/2;
%        0      0   1      0];

% Xd = [ 0    0     1.5   0;
%        .75   .75  .75   0;
%        .75  -.75  1     0;
%       0     0     1.5   0;
%       -.75  -.75  1     0;
%       -.75   .75  1     0;
%        0      0   1     0];

Xd = [ 0    0     1.5    0;
    0    0     1      0;
    0    0     1.5    0;
    0    0     1      0;
    0    0     1.5    0;
    0    0     1      0];

cont = 1;    % counter



angulos = [];
XX = [];             % position data
kopt = 0;   % counter
kdrone = 0; % counter

% Time variables
tsim = 100;    % Simulation duration [s]
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
        
        % Desired Position
        if toc(t) > cont*10
            
            cont = cont + 1;
            
            
        end
        
        if cont<=size(Xd,1)
            A.pPos.Xd(1) = Xd(cont,1);   % x
            A.pPos.Xd(2) = Xd(cont,2);   % y
            A.pPos.Xd(3) = Xd(cont,3);   % z
            A.pPos.Xd(6) = Xd(cont,4);   % psi
        end
        
        % ----------------------------------------------------------
        
        %  Get current rigid body information from optitrack
        rb      = OPT.RigidBody;
        A       = getOptData(rb(idA),A);
        A.pSC.U = [A.pPos.X(4);A.pPos.X(5);A.pPos.X(9);A.pPos.X(12)]; % populates actual control signal to save data
        
        
        % Control
        A = cUnderActuatedController(A);
        A = J.mControl(A);           % joystick command (priority)
        
        
        % Save data ------------------------------------------
        % Control signal
        lim = (15*pi/180);
        % U = [phi theta dz dpsi]
        A.pSC.U = [A.pPos.X(4)/lim; A.pPos.X(5)/lim; A.pPos.X(9); A.pPos.X(12)/(100*pi/180)];
        
        % text file
        fprintf(Arq,'%6.6f\t',[A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' toc(t)]);
        fprintf(Arq,'\n\r');
        
        % variable
        XX = [XX [A.pPos.Xd; A.pPos.X; A.pSC.Ud; A.pSC.U; toc(t)]];
        % ----------------------------------------------------------------
        
        A.rSendControlSignals;
        
    end
    
    % Draw robot
    if toc(tp) > 0.3
        tp = tic;
        A.mCADplot; hold on
        plot3(XX(1,:),XX(2,:),XX(3,:),'k-','LineWidth',0.5);
        plot3(XX(13,:),XX(14,:),XX(15,:),'g-','LineWidth',1);
        drawnow
    end
    
    % Timeout
    if toc(t)>tout
        disp('Time out. Ending simulation...');
        break
    end
    
end

% Land drone
if A.pFlag.Connected == 1
    A.rLand;
    %     A.rDisconnect;
end

kopt
kdrone
% % Close txt file
fclose(Arq);

%% Plot results
%
% figure
% plot(ta)

% Roll and pitch angles
figure
subplot(311),plot(XX(end,:),XX([4 16],:)'*180/pi)
legend('\phi_{Des}[^o]','\phi_{Atu}[^o]')
grid
subplot(312),plot(XX(end,:),XX([5 17],:)'*180/pi)
legend('\theta_{Des}[^o]','\theta_{Atu}[^o]')
grid
subplot(313),plot(XX(end,:),XX([6 18],:)'*180/pi)
legend('\psi_{Des}[^o]','\psi_{Atu}[^o]')
grid

% Trajectory 2D
figure
% axis equal
plot(XX([1,13],:)',XX([2,14],:)')
% axis([-1.5 1.5 -1.5 1.5])


% Trajectory 3D
figure
% axis equal
plot3(XX([1,13],:)',XX([2,14],:)',XX([3,15],:)')
% axis([-1.5 1.5 -1.5 1.5])
grid on


% X and Y
figure
subplot(311),plot(XX(end,:),XX([1 13],:)')
legend('x_{Des}','x_{Atu}')
grid
subplot(312),plot(XX(end,:),XX([2 14],:)')
legend('y_{Des}','y_{Atu}')
grid
subplot(313),plot(XX(end,:),XX([3 15],:)')
legend('z_{Des}','dz_{Atu}')
grid

% velocities
figure
subplot(311),plot(XX(end,:),XX([7 19],:)')
legend('dx_{Des}','dx_{Atu}')
axis([0 XX(end,end) -1 1])
grid
subplot(312),plot(XX(end,:),XX([8 20],:)')
legend('dy_{Des}','dy_{Atu}')
axis( [0 XX(end,end) -1 1])
grid
subplot(313),plot(XX(end,:),XX([9 21],:)')
legend('dz_{Des}','dz_{Atu}')
axis([0 XX(end,end) -1 1])
grid


