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

% Robot initialization
A = ArDrone;
L = Load;
O(1) = Obstacle;
O(2) = Obstacle;

L.pPar.l = 873/1000;
L.pPar.m = 0.05;
L.pPos.X(4:5) = [0 0];

O(1).pPos.X(1:3) = [ -240 +613 2000]/1000;
O(2).pPos.X(1:3) = [ +458 -817 2000]/1000;

gains = [1.2 2.7 1.2 2.7 2 15;
         2 17 2 17 1 5];

A.pPar.uSat(1:2) = A.pPar.uSat(1:2)*2;
A.pPar.uSat(3) = A.pPar.uSat(3)*1.3;

ForceFieldPar.Forc = [0.00 .015];
ForceFieldPar.Dist = [0.5 1.2];
%% Open file to save data
% DroneVar = CreateDataFile(A,'Sim','Log_Optitrack');
OptData = CreateOptDataFile(A,L,'TesteCarga','Log_Optitrack');
% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
%% Initialize classes
% Connect Joystick
J = JoyControl;

%% % Moviment o em 
Xd = [-1.5   0.0   1.3    10.0;
      +1.6   0.0   1.3    30.0;
      -1.5   0.0   1.3    45.0];
%%
time = 12;  % target hover time  [s]
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
A.pPos.Xa(1:6) = A.pPos.X(1:6);

A.rTakeOff;

% Time variables
tsim = Xd(end,4);        % Simulation time [s]
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
        if toc(t) > Xd(cont,4)
            cont = cont + 1;
        end
        
        A.pPos.Xd(1) = Xd(cont,1);   % x
        A.pPos.Xd(2) = Xd(cont,2);   % y
        A.pPos.Xd(3) = Xd(cont,3);   % z
%         A.pPos.Xd(6) = Xd(cont,4);
        A.pPos.Xd(6) = 0;
        
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
        
        if rb(2).isTracked
            dt = toc(td);
            if dt > 2/30
                dt = 1/30;
            end
            L = getOptData(rb(2),L);
            td = tic;
            kopt = kopt+1;
        end
        
%         % Filtro de sinal
        A.pPos.X = A.pPos.Xa*0.1 + A.pPos.X*0.9;
        L.pPos.X = L.pPos.Xa*0.1 + L.pPos.X*0.9;
        
        LD = dLoadDisturbance(L);
        D1 = sPotentialField(O(1),A,ForceFieldPar);
        D2 = sPotentialField(O(2),A,ForceFieldPar);
        PF = D1+D2;
        
        A.pPar.D = [LD' 0 0 0 PF 0]';
        disp([PF LD'] )
        A = cUnderActuatedControllerMexido(A,gains);
        A = J.mControl(A);           % joystick command (priority)
        
        % Control signal
        lim = (15*pi/180);
        % U = [phi theta dz dpsi]
        A.pSC.U = [A.pPos.X(4)/lim; A.pPos.X(5)/lim; A.pPos.X(9); A.pPos.X(12)/(100*pi/180)];
        
        % Save data ------------------------------------------
        SaveOptData(OptData,A,L,toc(t));
        
        % Variable
        XX = [XX [A.pPos.Xd; A.pPos.X; A.pSC.Ud; A.pSC.U; L.pPos.X; toc(t)]];
        % ----------------------------------------------------------------
        
        A.rSendControlSignals;
        
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


%% Plot results
close all
% Roll and pitch angles
figure(1)
subplot(311),plot(XX(end,:),XX([4 16 36],:)'*180/pi); grid; ylabel('\psi [^o]'); title('Angles')
subplot(312),plot(XX(end,:),XX([5 17 37],:)'*180/pi); grid; ylabel('\psi [^o]'); 
subplot(313),plot(XX(end,:),XX([6 18 38],:)'*180/pi); grid; ylabel('\psi [^o]'); xlabel('t [s]')
legend(' _{Des}',' _{Atu}',' _{Load}')

%% Trajectory 2D
figure(2)
plot(XX([1,13],:)',XX([2,14],:)'); grid;
axis([-1 1 -1 1]*2)
ylabel('y [m]'); xlabel('x [m]')

%% Trajectory 3D
figure(3)
plot3(XX([1,13 33],:)',XX([2,14 34],:)',XX([3,15 35],:)'); grid;
axis([-2 2 -2 2 0 1]*2)
zlabel('z [m]'); ylabel('y [m]'); xlabel('x [m]')

%% X and Y
figure(4)
subplot(311),plot(XX(end,:),XX([1 13 33],:)'); grid; ylabel('x [m]'); title('Coordinates');
subplot(312),plot(XX(end,:),XX([2 14 34],:)'); grid; ylabel('y [m]');
subplot(313),plot(XX(end,:),XX([3 15 35],:)'); grid; ylabel('z [m]'); xlabel('t [s]')
legend(' _{Des}',' _{Atu}',' _{Load}')
%%
% velocities
figure(5)
subplot(311),plot(XX(end,:),XX([7 19],:)'); grid; ylabel('dx [m/s]'); title('Velocities');
axis([0 XX(end,end) -1 1])
subplot(312),plot(XX(end,:),XX([8 20],:)'); grid; ylabel('dy [m/s]');
axis( [0 XX(end,end) -1 1])
subplot(313),plot(XX(end,:),XX([9 21],:)'); grid; ylabel('dz [m/s]'); xlabel('t [s]')
legend(' _{Des}',' _{Atu}')
axis([0 XX(end,end) -1 1])




disp(['Battery Level: ' num2str(A.pPar.Battery) '%'])
% CloseDataFile(DroneVar);
CloseOptDataFile(OptData);
