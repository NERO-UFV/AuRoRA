function iParameters(obj)

    obj.pPar.Model = 'Pioneer3DX'; % robot type
    obj.pPar.ip = ['192.168.1.' num2str(obj.pID)];
    % drone.pPar.LocalPortControl = 5556;
    % drone.pPar.LocalPortState = 5554;

    % Sample time
    obj.pPar.t   = tic;  % Current Time
    obj.pPar.Ts  = 1/10;  % Bebop
    obj.pPar.ti  = tic;    % Flag time 
    obj.pPar.tvel  = tic;

    obj.pPar.Tsm = obj.pPar.Ts/4; % Motor

    % Dynamic Model Parameters 
    obj.pPar.g = 9.8;         % [kg.m/s^2] Gravitational acceleration


    % Saturation values
    %     pitch          | [-1,1] <==> [-35,35] degrees
    %     roll           | [-1,1] <==> [-35,35] degrees
    %     altitude rate  | [-1,1] <==> [-6,6] m/s
    %     yaw rate       | [-1,1] <==> [-200,200] degrees/s
    %     Linear Speed   | [-1,1] <==> [-16,16] m/s
    %     Vertical Speed | [-1,1] <==> [-6,62]  m/s
    % drone.pPar.uSat    = zeros(6,1);
    % drone.pPar.uSat(1) = 35*pi/180;  % Max roll  angle reference 
    % drone.pPar.uSat(2) = 35*pi/180;  % Max pitch angle reference 
    % drone.pPar.uSat(3) = 200*pi/180; % Max yaw rate reference 
    % drone.pPar.uSat(4) = 16;         % Max X Speed reference
    % drone.pPar.uSat(5) = 16;         % Max Y Speed reference
    % drone.pPar.uSat(6) = 6;          % Max Z Speed reference 

    obj.pPar.uSat    = zeros(6,1);
    obj.pPar.uSat(1) = 1;  % Max roll  angle reference 
    obj.pPar.uSat(2) = 1;  % Max pitch angle reference 
    obj.pPar.uSat(3) = 1;  % Max yaw rate reference 
    obj.pPar.uSat(4) = 1;  % Max X Speed reference
    obj.pPar.uSat(5) = 1;  % Max Y Speed reference
    obj.pPar.uSat(6) = 1;  % Max Z Speed reference 

end


