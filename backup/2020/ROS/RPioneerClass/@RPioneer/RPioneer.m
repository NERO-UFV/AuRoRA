classdef RPioneer < handle
    % In a methods block, set the method attributes
    % and add the function signature
    properties
        
        % Properties or Parameters
        pCAD   % Pioneer 3DX 3D image
        pPar   % Parameters
        pID    % Identification
        vrep   % V-Rep library
        
        % Control variables
        pPos   % Posture        
        pSC    % Signals
        pFlag  % Flags
        
        % Navigation Data and Communication
        pData % Flight Data
        pCom  % Communication
        pOdom % Odometry Data 
        pPot1 % Potentiometer Data
        pPot2 % Potentiometer Data
        
        % Status variables
        pMotors
        
        % ROS Parameters        
        % Global Parametrs                
        pWorkSpaceName  % Standard 'bebop_ws'
        pNamespace
        iNamespace
        pTxtTopic        
                       
        % Standards Messages        
        pVel
        pStdMsgEmpty        
        pStdMsgGeoTwi
        
        % Standards Publishers
        pubCmdVel
        
        % Standards Listeners   
        subOdom   
        subOdomOpt
        subSonar
        subPot1
        subPot2
        
        % Services
        serEnaMotor
        serDisMotor
        
    end
    
    methods
        function obj = RPioneer(ID,iNamespace,flagTrailer)
            if nargin < 3 && nargin == 2
                flagTrailer = 0;
            end  
            
            if nargin < 2 && nargin == 1
                iNamespace = 'P';
                flagTrailer = 0;
            end  
            
            if nargin < 1
                ID = 1;
                iNamespace = 'P';
                flagTrailer = 0;
            end                                    
            obj.pID = ID;
            
            mInit(obj,ID,iNamespace,flagTrailer);
                
            mCADmake(obj);
      
        end
        
        % ==================================================
        iControlVariables(obj);
        iParameters(obj);
        iFlags(obj);       
        
        % ==================================================
        % Pioneer 3DX 3D Image
        mCADmake(obj);
        mCADplot(obj,scale,color);
        mCADplot2D(obj,visible);
        mCADdel(obj);
        % mCADcolor(obj,color);
        
        % ==================================================
        % Pose definition, based on kinematic or dynamic model
        sKinematicModel(obj);
        sInvKinematicModel(obj,dXr);
        sDynamicModel(obj);
        
        % ==================================================
        % Robot functions
        % Communication
        rConnect(obj);
        rDisconnect(obj);
        rSetPose(obj,Xo);
        % Data request
        rGetSensorData(obj);
        rGetSensorDataOpt(obj);
        dist = rGetSonarData(obj);
        
        % Command
        rSendControlSignals(obj);
        rCmdStop(obj);
        rCmdVel(obj);
        rCommand(obj);
        rEnableMotors(obj);
        rDisableMotors(obj);
        
        
        % ==================================================
      
    end
end