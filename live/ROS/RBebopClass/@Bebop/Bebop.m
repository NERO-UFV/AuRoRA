%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:


classdef Bebop < handle
    
    properties
        
        % Properties or Parameters
%         pCAD   % ArDrone 3D image
        pPar   % Parameters Dynamic Model
        pID    % Identification
                
        % Control variables
        pPos      % Posture
        pSC       % Signals
        pFlag     % Flags        
                
        % Navigation Data and Communication
        pData % Flight Data
        pCom  % Communication        
        pOdom % Odometry Data  
        Marker
        
        % ROS Parameters
        
        % Global Parametrs                
        pWorkSpaceName  % Standard 'bebop_ws'
        pNamespace      % Standard 'bebop'
        pTxtTopic
        
                       
        % Standards Messages        
        pVel
        pStdMsgEmpty        
        pStdMsgGeoTwi
        
        % Standards Publishers
        pubCmdVel
        pubBrTakeoff
        pubBrLand
        
        % Standards Listeners   
        subOdom
        subOdomLocal
        subOdomOpt
        subLand
        subBattery
        subMarker
        
    end
    methods
        function obj = Bebop(ID,iNamespace)
            mInit(obj,ID,iNamespace); % Initialize variables            
        end   
        
        function Callback_EmgStop(obj,src,message)
            obj.pFlag.EmergencyStop = 1;
        end
                    
        % Takeoff/Landing
        rTakeOff(obj);        
        rLand(obj);                
        
        % Controllers
        cInverseDynamicController(obj,model,gains)
        cInverseDynamicController_Milton(obj,model,gains)
        cInverseDynamicController_Compensador(obj,gains)
        cInverseDynamicController_Compensador_mod(obj,gains,L)

		cInverseDynamicController_Adaptativo(obj,gains)
        
        
        
        % Set Methods
        rSetGeometryMsg(obj,twistObj);
        rSetGeometryMsgVar(obj);        
        msgObj = mSetTwistObj(msgObj,Lx,Ly,Lz,Ax,Ay,Az);
                
                
        % Get Methods
        
                
        % Send Methods        
        rCmdStop(obj);
        rCmdVel(obj,iVar);                       
                
        % Send Commands
        rCommand(obj);
        rSetLed(obj,id,freq,duration);
        rSendControlSignals(obj);
                
        % Data request
        rGetSensorData(obj);   
        rGetSensorDataLocal(obj);
        rGetSensorDataOpt(obj);
		rGetLastSensorDataOpt(obj);
        rGetSensorOdomMsg(obj);
        rGetMarker(obj);
                             
        % ==================================================
        % obj functions
        % Communication
        rConnect(obj);
        rDisconnect(obj);        

        % Emergency
        rEmergency(obj)            
        
        % ==================================================
        % ArDrone 3D Image
%         mCADload(obj);
%         mCADcolor(obj,cor);
%         mCADplot(obj,visible);
%         mCADdel(obj);
        
        % ==================================================
        iControlVariables(obj);
        iParameters(obj);
        
        % ==================================================
        sDynamicModel(obj);

         
    end
end