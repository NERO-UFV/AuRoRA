clear; close all; clc;
try
    fclose(instrfindall);
catch
end

% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Class
try
    % Load Classes
    RI = RosInterface;
    setenv('ROS_IP','192.168.0.153')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    P1 = RPioneer(1,'P1');
    P2 = RPioneer(1,'P2');
    P3 = RPioneer(1,'P3');
    
    disp('################### Load Class Success #######################');
    
catch ME
    disp(' ');
    disp(' ################### Load Class Issues #######################');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    rosshutdown;
    return;
    
end

P1.rGetSensorDataOpt;
P2.rGetSensorDataOpt;
P3.rGetSensorDataOpt;

t = tic;
tc = tic;
ta = 0.1;
tmax = 60;

K = diag([.5 .5]);

a_P = 0.15;

while toc(t) < tmax
    if toc(tc) > ta
        tc = tic;
        P1.rGetSensorDataOpt;
        P2.rGetSensorDataOpt;
        P3.rGetSensorDataOpt;
        
        X1 = P1.pPos.X;
        X2 = P2.pPos.X;
        X3 = P3.pPos.X;
        
        Xd1 = [-1 1]';
        dXd1 = [0 0]';
        
        Xd2 = [1 1]';
        dXd2 = [0 0]';
        
        Xd3 = [1 -1]';
        dXd3 = [0 0]';
        
        Xtil1 = Xd1(1:2) - X1(1:2);
        Xtil2 = Xd2(1:2) - X2(1:2);
        Xtil3 = Xd3(1:2) - X3(1:2);
        
        dXref1 = dXd1 + K*Xtil1;
        dXref2 = dXd2 + K*Xtil2;
        dXref3 = dXd3 + K*Xtil3;
        
        F_P1 = [cos(X1(6)) -a_P*sin(X1(6));
            sin(X1(6)) a_P*cos(X1(6))];
        
        F_P2 = [cos(X2(6)) -a_P*sin(X2(6));
            sin(X2(6)) a_P*cos(X2(6))];
        
        F_P3 = [cos(X3(6)) -a_P*sin(X3(6));
            sin(X3(6)) a_P*cos(X3(6))];
        
        Vref1 = F_P1\dXref1;
        Vref2 = F_P2\dXref2;
        Vref3 = F_P3\dXref3;
        
        P1.pSC.Ud = Vref1;
        P2.pSC.Ud = Vref2;
        P3.pSC.Ud = Vref3;

        P1.rCommand;
        P2.rCommand;
        P3.rCommand;
    end
end