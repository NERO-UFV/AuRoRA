clear all
close all
clc

%%

try
    fclose(instrfindall);
catch
end

%% Load Class
try
    % Load Classes
    RI = RosInterface;
    setenv('ROS_IP','192.168.0.158')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    
    % Create OptiTrack object and initialize
    OPT = OptiTrack;
    OPT.Initialize;
    
    nPioneer = 1;
    ID = 1;
    
    for ii = 1:nPioneer
        P{ii} = RPioneer(ii,['P' num2str(ii)]);
        % detect rigid body ID from optitrack
        idP{ii} = ii;
    end    
        
    % Joystick
    J = JoyControl;
    Joy = 1;
    
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

for ii = 1:nPioneer
    P{ii}.pPar.a = 0;
    P{ii}.pPar.alpha = 0;

    rb = OPT.RigidBody;          % read optitrack data
    % B = getOptData(rb(idB),B);   % get ardrone data
    try
    P{ii} = getOptData(rb(idP{ii}),P{ii});   % get pioneer data
    end
    P{ii}.rEnableMotors;
end
%% CONTROLLER GAINS
pgains = [0.1 0.1 1];

%% TEMPORIZADORES
T_AMOSTRAGEM = P{1}.pPar.Ts;

Ta = tic;

while J.pFlag == 0 && Joy == 1
if toc(Ta) > T_AMOSTRAGEM 
    Ta = tic;
    
    % Acquire sensors data        
    % Get optitrack data        
    rb = OPT.RigidBody;             % read optitrack
    
    try
        if rb(idP{ii}).isTracked
            P{ID} = getOptData(rb(idP{ID}),P{ID});
            clc
            disp(P{ID}.pPos.X([1 2 6]))
        end
    catch
    end
    
    % Change routine
    if J.pSwitch ~= 0
        ID = ID + J.pSwitch;
        %         disp(J.pSwitch)
        J.pSwitch = 0;
        
        if ID > nPioneer
            ID = 1;
        end
        
        if ID < 1
            ID = nPioneer;
        end
        disp(ID)
    end
    
    P{ID} = J.mControl(P{ID});
    
    P{ID}.rCommand;
    
    P{ID}.pSC.Ud = [0; 0];
end
end

for ii = 1:nPioneer
    P{ii}.rDisableMotors;
%     P{ii}.rEnableMotors;
end


















