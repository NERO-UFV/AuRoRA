%% GET DATA FROM TWO RIGID BODIES (FORMATION DRONE-PIONEER)
% Initialization
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

%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_Optitrack')
Arq = fopen(['teste_drone_' NomeArq '.txt'],'w');
cd(PastaAtual)

%% Initialize classes
% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
%OPT.Initialize('192.168.0.1');
% Robot initialization
P = Pioneer3DX;
A = ArDrone(2);

%% Window definition
figure(1)
axis([-3 3 -3 3 0 3])
grid on
% A.mCADplot;
% P.mCADplot(1,'k');
drawnow

%% Variables Initialization

tp = tic;
%% Simulation loop
while true
    
    %  Get current rigid body information from optitrack
    rb = OPT.RigidBody;
    n = length(rb);
    if ~isempty(n)
        for ii = 1:n
            switch rb(ii).Name(1)
                case 'P'
                    P = getOptData(rb(ii),P);
                case 'A'
                    A = getOptData(rb(ii),A);
                otherwise
                    disp('Unknown rigid body type. (Known types: "A" or "P")');
            end
        end
        
    else
        disp('No rigid body tracked');
    end
    

    % Draw robot
    if toc(tp) > 0.3
        try
            P.mCADdel
            delete(fig);
        catch
        end
        tp = tic;
        A.mCADplot;
        P.mCADplot(1,'k');
        drawnow
    end
    
    %     % Timeout
    %     if toc(t)>tout
    %         disp('Time out. Ending simulation...');
    %         break
    %     end
    
end

