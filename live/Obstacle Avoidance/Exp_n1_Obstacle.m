%% Test to create a load in the ambient

% Inicialização
close all; clear; clc

try
    fclose(instrfindall);
catch
end

% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

% Robot initialization
A = ArDrone;
drone.pPar.Cf = 1.14298e-9;
drone.pPar.Ct = 3.12e-12;
% L = Load;
O = Obstacle;
O.pPos.X(1:3) = [0 0 1.5];
ForceFieldPar.Forc = [0.00 0.008];
ForceFieldPar.Dist = [0.99 1.1];

gains = [1.2 2.6 1.2 2.6 2.2 15;
         2 15 2 15 1 10];

A.pPar.uSat(1:2) = A.pPar.uSat(1:2)*2.0;
A.pPar.uSat(3) = A.pPar.uSat(3)*1.0;
A.pPar.uSat(4) = A.pPar.uSat(4)*1.0;

OptData = CreateOptDataFile(A,'DesvioObs','Log_Optitrack');

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

%% Initialize classes
% Connect Joystick
J = JoyControl;
%% Variables Initialization
% Desired positions
Xd = [-1.5   0.0   1 0 10.0;
      +1.3   0.0   1 0 30.0;
      -1.5   0.0   1 0 40.0];

time = 15;  % target hover time  [s]
cont = 1;    % counter

angulos = [];
XX = [];             % position data
kopt = 0;   % counter
kdrone = 0; % counter

%%
A.rConnect;
A.rGetSensorData;
disp(['Initial Battery Level: ' num2str(A.pPar.Battery) '%'])
%%
rb = OPT.RigidBody;
A = getOptData(rb(1),A);
% O = getOptData(rb(2),O);
% O = getOptData(rb,O);
% O.pPos.X(3) = 2;
% O.mCADCreate

A.pPos.Xa(1:6) = A.pPos.X(1:6);
%%
A.rTakeOff;

% Time variables
% tsim = size(Xd,1)*time;   % Simulation time [s]
tsim = Xd(end,5);        % Simulation time [s]
tout = 100;              % maximum simulation duration [s]
tc = tic;                % drone frequency
tp = tic;                % graph refresh rate
tt = tic;                % trajectory time
t = tic;                 % simulation current time
td = tic;
ta = [];
dt = 1/30;

%% Simulation loop
while toc(t) < tsim
    if toc(tc) > A.pPar.Ts
        ta = [ta toc(tc)];
        tc = tic;
        
        % Desired Position
        if toc(t) > Xd(cont,5)
            cont = cont + 1;
        end
        
        A.pPos.Xd(1) = Xd(cont,1);   % x
        A.pPos.Xd(2) = Xd(cont,2);   % y
        A.pPos.Xd(3) = Xd(cont,3);   % z
        A.pPos.Xd(6) = Xd(cont,4); % A.pPos.Xd(6) = Xd(cont,4); % Psi
        
        % ----------------------------------------------------------
        
        %  Get current rigid body information from optitrack
        rb = OPT.RigidBody;
        
        if rb(1).isTracked
            dt = toc(td);
            if dt > 2/30
                dt = 1/30;
            end
            A = getOptData(rb(1),A);
            td = tic;
            kopt = kopt+1;
        end
        
        % Controlador
        PF = sPotentialField_New(O,A,ForceFieldPar);
        A.pPar.D = [0 0 0 PF' 0]';
        disp(PF')
        A = cUnderActuatedControllerMexido(A,gains);
        A = J.mControl(A); % joystick command (priority)

        
        % Control signal
        lim = (15*pi/180);
        % U = [phi theta dz dpsi]
        A.pSC.U = [A.pPos.X(4)/lim; A.pPos.X(5)/lim; A.pPos.X(9); A.pPos.X(12)/(100*pi/180)];
        % Save data ------------------------------------------
        SaveOptData(OptData,A,O,toc(t));
        
        % Variable
        XX = [XX [A.pPos.Xd; A.pPos.X; A.pSC.Ud; A.pSC.U; O.pPos.X; toc(t)]];
        % ----------------------------------------------------------------
        
        A.rSendControlSignals;
    end
    
    % Matriz de Coriolis e Vetor de Forças Gravitacionais Ativa

    
end

% Land drone
if A.pFlag.Connected == 1
    A.rLand;
    % A.rDisconnect;
end

kopt
kdrone
% % Close txt file

%% Plot results
close all
if false
    % Roll and pitch angles
    figure(1)
    subplot(311),plot(XX(end,:),XX([4 16 36],:)'*180/pi); grid; ylabel('\psi [^o]'); title('Angles')
    subplot(312),plot(XX(end,:),XX([5 17 37],:)'*180/pi); grid; ylabel('\psi [^o]');
    subplot(313),plot(XX(end,:),XX([6 18 38],:)'*180/pi); grid; ylabel('\psi [^o]'); xlabel('t [s]')
    legend(' _{Des}',' _{Atu}',' _{Load}')
end
%% Trajectory 2D
figure(2)
plot(XX([1,13],:)',XX([2,14],:)'); grid;
axis([-1 1 -1 1]*2)
ylabel('y [m]'); xlabel('x [m]')
O.mCADCreate
%% Trajectory 3D
if false
    %%
    figure(3)
    plot3(XX([1,13],:)',XX([2,14],:)',XX([3,15],:)'); grid;
    axis([-2 2 -2 2 0 1]*2)
    zlabel('z [m]'); ylabel('y [m]'); xlabel('x [m]')
    O.mCADCreate
end
%% X and Y
if true
    figure(4)
    subplot(311),plot(XX(end,:),XX([1 13 33],:)'); grid; ylabel('x [m]'); title('Coordinates');
    subplot(312),plot(XX(end,:),XX([2 14 34],:)'); grid; ylabel('y [m]');
    subplot(313),plot(XX(end,:),XX([3 15 35],:)'); grid; ylabel('z [m]'); xlabel('t [s]')
    legend(' _{Des}',' _{Atu}',' _{Load}')
end
%%
% velocities
if false
    figure(5)
    subplot(311),plot(XX(end,:),XX([7 19],:)'); grid; ylabel('dx [m/s]'); title('Velocities');
    axis([0 XX(end,end) -1 1])
    subplot(312),plot(XX(end,:),XX([8 20],:)'); grid; ylabel('dy [m/s]');
    axis( [0 XX(end,end) -1 1])
    subplot(313),plot(XX(end,:),XX([9 21],:)'); grid; ylabel('dz [m/s]'); xlabel('t [s]')
    legend(' _{Des}',' _{Atu}')
    axis([0 XX(end,end) -1 1])
end
%%
% CloseDataFile(DroneVar);
CloseOptDataFile(OptData);


