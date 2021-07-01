classdef Catamara < handle
    
    properties
        
        % Properties or Parameters
        pCAD   % Arcatamara 3D image
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
        catamaraHandle              % Pioneer Tag on V-Rep
        MotorHandle = [0;0;0;0]; % Motors Tags, Left Motor and Right Motor
        GPS_handle               % GPS tag
           
        
    end
    methods
        function catamara = Catamara(ID)
            if nargin < 1
                ID = 1;
            end
            
            catamara.pID = ID; 
            
            catamara.pFlag.Connected = 0;    
            
            iControlVariables(catamara);
            iParameters(catamara);
            
            mCADload(catamara);
            catamara.vrep = remApi('remoteApi');
        end
        
        % ==================================================
        % catamara functions
        % Communication
        rConnect(catamara);
        rDisconnect(catamara);
        
        % Data request
        rGetStatusRawData(catamara);
        rGetStatusRawDataFull(catamara);
        rGetSensorData(catamara);
        rGetSensorCalibration(catamara);
        rGetIp(catamara);
        
        % Takeoff/Landing
        rTakeOff(catamara);
        rTakeOffMultiples(catamara1,catamara2);
        rLand(catamara);
        
        % Emergency
        rEmergency(catamara)
        
        % Command
        rCommand(catamara);
        rSetLed(catamara,id,freq,duration);
        rSendControlSignals(catamara);
        
        
        % ==================================================
        % Arcatamara 3D Image
        mCADload(catamara);
        mCADcolor(catamara,cor);
        mCADplot(catamara,visible);
        mCADdel(catamara);
        
        % ==================================================
        iControlVariables(catamara);
        iParameters(catamara);
        
        % ==================================================
        sDynamicModel(catamara);
        
        %% Robot functions
        % Communication V-Rep
        vConnect(catamara);
        vDisconnect(catamara);
        
        %Handle Objects
        vHandle(catamara,robot_name,index);
%         
        % Data Request
        vGetSensorData(catamara);
        % Send Command
        vSendControlSignals(catamara);
        vSetPose(catamara,vector);
        
        
    end
end