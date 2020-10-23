%% Inicialização

close all
clear
clc

try
    fclose(instrfindall);
catch
end

% Search root directory
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Robot initialization
P = Pioneer3DX;
P.pPar.a = 0.005;       % control point

% open serial port
s = serial('COM3');
fopen(s);

%% detect rigid body ID from optitrack
idP = getID(OPT,P);              % pioneer ID on optitrack

% initial positions
rb = OPT.RigidBody;              % read optitrack data
P = getOptData(rb(idP),P);       % get pioneer data
P.pPar.Ts = 1/60;

%% Parameters
data = [];                       % variable to save positions, velocities and time
vel  = 0;
vmin = 45;                       % minimum velocity to move the robot
vl   = 116;                      % left side PWM
vr   = 116;                      % right side PWM
vmax = 116;                      % maximum velocity
tsim = 4;                        % simulation time
tp   = tic;                      % transmission rate
t    = tic;                      % current time
%% Simulation loop
while toc(t) < tsim
    
    if toc(tp)>1/60
        tp = tic;
        %  Get current rigid body information from optitrack
        rb = OPT.RigidBody;
        P = getOptData(rb(idP),P);  % get pioneer data
        
        % Start with maximum velocity
        if toc(t)<1.5
            % Write and send PWM to robot
            pacote=(['B' 'D' 22  vmax vmax 116 116 116 116 'P']);
            fprintf(s,pacote);
            
        else
            
            % Write and send PWM to robot
            pacote=(['B' 'D' 22  vl vr 116 116 116 116 'P']);
            fprintf(s,pacote);
        end
        % Save data
        data = [data [P.pPos.X ; vl ; vr ; toc(t)]];
        
    end
    
    % Draw robot
    if toc(tp) > 0.1
        
        P.mCADdel
        Bola.mCADdel
        
        tp = tic;
        P.mCADplot(0.2,'k');
        Bola.mCADplot(0.2,'b');
        axis([-1 1 -1 1])
        drawnow
        grid on
    end
    
end

% Save variable
save('C:\Users\NERO_2\Dropbox\AuRoRA 2018\DataFiles\Log_bdp\PWM','data');
% close serial port
fclose(s);

% plot some results
plot(data(end,:),data(7,:))
ylabel('Velocidade x [m/s]')
grid on
figure;
plot(data(end,:),data(8,:))
ylabel('Velocidade y [m/s]')
grid on

figure;
plot(data(end,:),data(12,:))
ylabel('Velocidade angular [rad/s]')
grid on

