classdef ArDrone < handle
    
    properties
        
        % Properties or Parameters
        pCAD   % ArDrone 3D image
        pPar   % Parameters Dynamic Model
        pID    % Identification
        vrep   % V-Rep library
        
        % Control variables
        pPos   % Posture
        pSC    % Signals
        pFlag  % Flags
        
        % Navigation Data and Communication
        pData % Flight Data
        pCom  % Communication
        
        % V-Rep Navigation and Communication
        robot_name               % Robot name for the handles
        clientID                 % Client Server Identification
        DroneHandle              % Pioneer Tag on V-Rep
        MotorHandle = [0;0;0;0]; % Motors Tags, Left Motor and Right Motor
        GPS_handle               % GPS tag
           
        
    end
    methods
        function drone = ArDrone(ID)
            if nargin < 1
                ID = 1;
            end
            
            drone.pID = ID; 
            
            drone.pFlag.Connected = 0;    
            
            iControlVariables(drone);
            iParameters(drone);
            
            mCADload(drone);
            drone.vrep = remApi('remoteApi');
        end
        
        % ==================================================
        % Drone functions
        % Communication
        rConnect(drone);
        rDisconnect(drone);
        
        % Data request
        rGetStatusRawData(drone);
        rGetStatusRawDataFull(drone);
        rGetSensorData(drone);
        rGetSensorCalibration(drone);
        rGetIp(drone);
        
        % Takeoff/Landing
        rTakeOff(drone);
        rTakeOffMultiples(drone1,drone2);
        rLand(drone);
        
        % Emergency
        rEmergency(drone)
        
        % Command
        rCommand(drone);
        rSetLed(drone,id,freq,duration);
        rSendControlSignals(drone);
        
        
        % ==================================================
        % ArDrone 3D Image
        mCADload(drone);
        mCADcolor(drone,cor);
        mCADplot(drone,visible);
        mCADdel(drone);
        
        % ==================================================
        iControlVariables(drone);
        iParameters(drone);
        
        % ==================================================
        sDynamicModel(drone);
        
        %% Robot functions
        % Communication V-Rep
        vConnect(drone);
        vDisconnect(drone);
        
        %Handle Objects
        vHandle(drone,robot_name,index);
%         
        % Data Request
        vGetSensorData(drone);
        % Send Command
        vSendControlSignals(drone);
        vSetPose(drone,vector);
        
        
    end
end