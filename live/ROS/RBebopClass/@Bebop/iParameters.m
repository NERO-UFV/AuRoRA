function iParameters(obj)

    obj.pPar.Model = 'Bebop2'; % robot type
    obj.pPar.ip = ['192.168.0.' num2str(obj.pID)];

    % Sample time
    obj.pPar.t   = tic;  % Current Time
    obj.pPar.Ts  = 1/5; % Default Step Time
    obj.pPar.ti  = tic;    % Flag time 
    obj.pPar.tder  = tic;    % Flag time 

    obj.pPar.Tsm = obj.pPar.Ts/1; % Motor

    % Dynamic Model Parameters 
    obj.pPar.g = 9.8;         % [kg.m/s^2] Gravitational acceleration
    obj.pPar.Altmax = 2000;   % Max Altitude Allowed
    obj.pPar.Model_simp = zeros(8,1);
    
    % Defaut Controller Gains 
    obj.pPar.L1 = diag([1 1 1 1 1 1]);
    obj.pPar.L2 = diag([1 1 .5 .5 .5 .5]);
    obj.pPar.K  = diag([.8 .8 .8 .8]);
    
    % Defaut Drone Dynamic Model
    obj.pPar.Ku = diag([0.8417 0.8354 3.966 9.8524]);
    obj.pPar.Kv = diag([0.18227 0.17095 4.001 4.7295]);           

    % Saturation values
    %     pitch          | [-1,1] <==> [-35,35] degrees
    %     roll           | [-1,1] <==> [-35,35] degrees
    %     altitude rate  | [-1,1] <==> [-6,6] m/s
    %     yaw rate       | [-1,1] <==> [-200,200] degrees/s
    %     Linear Speed   | [-1,1] <==> [-16,16] m/s
    %     Vertical Speed | [-1,1] <==> [-6,62]  m/s
    obj.pPar.uSat    = zeros(6,1);
    obj.pPar.uSat(1) = 1;  % Max roll  angle reference 
    obj.pPar.uSat(2) = 1;  % Max pitch angle reference 
    obj.pPar.uSat(3) = 1;  % Max yaw rate reference 
    obj.pPar.uSat(4) = 1;  % Max X Speed reference
    obj.pPar.uSat(5) = 1;  % Max Y Speed reference
    obj.pPar.uSat(6) = 1;  % Max Z Speed reference 
    
    
    % Flags
    obj.pFlag.EmergencyStop = 0;
    obj.pFlag.Connected = 1;
    obj.pFlag.isTracked = 1;    

end



