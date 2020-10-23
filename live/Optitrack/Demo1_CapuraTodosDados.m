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
A.pPar.Ts = 1/30;
A.pPar.Cgains = [1.5 3.2 1.2 2 2 17;
                 2 15 2 15 1 5];

% A.pPar.Cgains = [.5 1.8 1 1.5 4.8 1.8;
%     1.2 15 1.1 8 1 1.5];

%% Open file to save data
% DroneVar = CreateDataFile(A,'Sim','Log_Optitrack');
OptData = CreateOptDataFile(A,'60Hz','Log_Optitrack');

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;


%% Initialize classes
% Connect Joystick
J = JoyControl;
%% Variables Initialization
% Desired positions
Xd = [ 0    0     1.3    0;
%        1    0     1.3    0;
       1    0.5   1.3    0;
       0    0.5   1.3    0;
      -0.5  0.5   1.3    0;
%       -0.5  0     1.3    0;
       0    0     1.3    0];

% Xd = [ 0    0     1.5   0;
%        .75   .75  .75   0;
%        .75  -.75  1     0;
%       0     0     1.5   0;
%       -.75  -.75  1     0;
%       -.75   .75  1     0;
%        0      0   1     0];

% % Moviment o em X
% Xd = [0   0   1.2   0;
%       1   0   1.2   0;
%      -.5   0   1.2   0;
%       0   0   1.2   0];

% % Moviment o em Y
% Xd = [0   0   1.2   0;
%       0   .5   1.2   0;
%       0   -.5   1.2   0;
%       0   0   1.2   0];

% % % Moviment o em Z
% Xd = [0   0   1.3   0;
%       0   0   1.5   0;
%       0   0   1.7   0;
%       0   0   1.5   0];


time = 12;  % target hover time  [s]
cont = 1;    % counter

angulos = [];
XX = [];             % position data
kopt = 0;   % counter
kdrone = 0; % counter

A.rConnect;
A.rGetSensorData;
disp(['Initial Battery Level: ' num2str(A.pPar.Battery) '%'])

rb = OPT.RigidBody;
A = getOptData(rb,A);
A.pPos.Xa(1:6) = A.pPos.X(1:6);

A.rTakeOff;

% Time variables
tsim = size(Xd,1)*time;   % Simulation time [s]
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
        if toc(t) > cont*time
            cont = cont + 1;
        end
        
        A.pPos.Xd(1) = Xd(cont,1);   % x
        A.pPos.Xd(2) = Xd(cont,2);   % y
        A.pPos.Xd(3) = Xd(cont,3);   % z
        A.pPos.Xd(6) = Xd(cont,4);
        
        % ----------------------------------------------------------
%         
%         A.rGetSensorData
%         UA = A.pPos.X(3);
%         
%         SaveData(DroneVar,A,toc(t));
%         A.rGetAngles;
        

        %  Get current rigid body information from optitrack
        rb = OPT.RigidBody;

            if rb.isTracked
                dt = toc(td);
                if dt > 2/60
                    dt = 1/60;
                end
                A = getOptData(rb,A);            
                td = tic;
                kopt = kopt+1;
            end
    
        % Filtro de sinal
        A.pPos.X = A.pPos.Xa*0.3 + A.pPos.X*0.7;
%         
        A = cUnderActuatedController(A);
        A = J.mControl(A);           % joystick command (priority)
        
        % Control signal
        lim = (15*pi/180);
        % U = [phi theta dz dpsi]
        A.pSC.U = [A.pPos.X(4)/lim; A.pPos.X(5)/lim; A.pPos.X(9); A.pPos.X(12)/(100*pi/180)];
        
        % Save data ------------------------------------------

        SaveOptData(OptData,A,toc(t));
        
        % Variable 
        XX = [XX [A.pPos.Xd; A.pPos.X; A.pSC.Ud; A.pSC.U; toc(t)]];
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
%
% figure
% plot(ta)

% Roll and pitch angles
figure(1)
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
figure(2)
% axis equal
plot(XX([1,13],:)',XX([2,14],:)')
% axis([-1.5 1.5 -1.5 1.5])

%%
% Trajectory 3D
figure(3)
% axis equal
plot3(XX([1,13],:)',XX([2,14],:)',XX([3,15],:)')
axis([-1.5 1.5 -1.5 1.5])
grid on
%%

% X and Y
figure(4)
subplot(311),plot(XX(end,:),XX([1 13],:)')
legend('x_{Des}','x_{Atu}')
grid
subplot(312),plot(XX(end,:),XX([2 14],:)')
legend('y_{Des}','y_{Atu}')
grid
subplot(313),plot(XX(end,:),XX([3 15],:)')
legend('z_{Des}','z_{Atu}')
grid
%%
% velocities
figure(5)
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
%%
disp(['Battery Level: ' num2str(A.pPar.Battery) '%'])
% CloseDataFile(DroneVar);
CloseOptDataFile(OptData);
