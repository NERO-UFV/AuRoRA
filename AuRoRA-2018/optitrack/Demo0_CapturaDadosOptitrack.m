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
A.pPar.Cgains = [.5 2 .5 2 5 2; 1 20 1 15 1 2.5];
%% Open file to save data
DroneVar = CreateDataFile(A,'Sim','Log_Optitrack');
OptData = CreateOptDataFile(A,'Sim','Log_Optitrack');

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

%% Window definition
figure(1)
axis([-3 3 -3 3 0 3])
grid on
A.mCADplot
drawnow
pause(5)
%%

cont = 1;    % counter

angulos = [];
XX = [];             % position data
kopt = 0;   % counter
kdrone = 0; % counter

% Time variables
tsim = 20;    % Simulation duration [s]
tout = 100;     % maximum simulation duration [s]
tc = tic;      % drone frequency
tp = tic;      % graph refresh rate
tt = tic;      % trajectory time
t = tic;       % simulation current time
td = tic;
ta = [];
dt = 1/30;
%%
while toc(t) < tsim
    if toc(tc) > 1/30
        ta = [ta toc(tc)];
        tc = tic;

        
        %  Get current rigid body information from optitrack
        rb = OPT.RigidBody;
        try
            if rb.isTracked
                dt = toc(td);
                if dt > 2/30
                    dt = 1/30;
                end
                A = getOptData(rb,A);
                A.pPos.X(7:9) = (A.pPos.X(1:3)-A.pPos.Xa(1:3))/dt;
                A.pPos.X(10:11) = (A.pPos.X(4:5)-A.pPos.Xa(4:5))/dt;
                if abs(A.pPos.X(6) - A.pPos.Xa(6)) > pi
                    if A.pPos.Xa(6) < 0
                        A.pPos.Xa(6) =  2*pi + A.pPos.Xa(6);
                    else
                        A.pPos.Xa(6) = -2*pi + A.pPos.Xa(6);
                    end
                end
                A.pPos.X(12) = (A.pPos.X(6)-A.pPos.Xa(6))/dt;
                
                A.pPos.Xa = A.pPos.X;
                td = tic;
                kopt = kopt+1;
            end
        catch
            kdrone = kdrone+1;
        end

        % Save data ------------------------------------------
        SaveData(DroneVar,A,toc(t));
        SaveOptData(OptData,A,toc(t));
%     disp(A.pPos.X)
    end
   
    
    % Timeout
    if toc(t)>tout
        disp('Time out. Ending simulation...');
        break
    end
    
end


CloseDataFile(DroneVar)
CloseOptDataFile(OptData)






















