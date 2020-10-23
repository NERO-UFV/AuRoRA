%% Controle de Formação Baseado em Espaço Nulo
% ICUAS 2019
% Mauro Sérgio Mafra

%% Referência de Modelo Convencional (Marcos)
% Resetar 
clear all;   
close all;
warning off; 
clc;

%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

try
    fclose(instrfindall);
end

%% Load Class
try     
    % Load Classes
    RI = RosInterface; 
    RI.rConnect;    
    B = Bebop(1); 
    
    disp('### Load Class Success');
    
catch ME
    disp(' #### Load Class Issues ');
    disp('');
    disp(ME);
    return;
end


% Time variables initialization
timeout = 60;   % maximum simulation duration
sampleTime = 1/30;

% timeout = size(Qd,1)*time + 30;
t  = tic;
tc = tic;
tp = tic;
t1 = tic;        % pioneer cycle
t2 = tic;        % ardrone cycle
index = 1;

cont = 1;        % counter to change desired position through simulation
time = 60;       % time to change desired positions [s]
landTime = 3;   % 10 Seconds to Lend

%B.rTakeOff;
pause(3);
disp('Proceeded to Loop Control!.....');

try 
    
    % Loop while error > erroMax
    while toc(t)< (time)
        
        if toc(tc) > sampleTime        
            tc = tic;                 
        
            %B.rGetSensorOdomMsg;
            B.rGetSensorData;
%             disp("Velocidades......");                
%             disp(B.pOdom.Twist.Twist.showdetails);
%             disp("   ");                
%             disp("Posições......");                
%             disp(B.pOdom.Pose.Pose.showdetails);
%         
        end
    end
    
    % Send Command to Land
    B.rLand
    
    % Continuous watch land variables
    t = tic;          
    while toc(t)< (landTime)
        
        if toc(tc) > sampleTime        
            tc = tic;                 
        
            B.rGetSensorOdomMsg;
            %B.rGetSensorData;
            disp(B.pOdom.Pose.showdetails);
        
        end
    end
    
    
    
catch ME
    
    disp('');
    disp(ME);
    disp('');
    
    % Send Land Command
    B.rLand;    
    
    % Fecha o cliente ROS
    RI.rDisconnect;
    %rosshutdown;
end



% Send 3 times Commands 1 second delay to Drone Land
for i=1:3
    B.rLand
    pause(1);
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;