classdef Pioneer3DX < handle
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
        
        % Formation
        pFormation % Formations ID
        
    end
    
    methods
        function p3dx = Pioneer3DX(ID)
            if nargin < 1
                ID = 1;
            end
            p3dx.pID = ID;
                
            iControlVariables(p3dx);
            iParameters(p3dx);
            iFlags(p3dx);
            mCADmake(p3dx);
      
        end
        
        % ==================================================
        iControlVariables(p3dx);
        iParameters(p3dx);
        iFlags(p3dx);       
        
        % ==================================================
        % Pioneer 3DX 3D Image
        mCADmake(p3dx);
        mCADplot(p3dx,scale,color);
        mCADplot2D(p3dx,visible);
        mCADdel(p3dx);
        % mCADcolor(p3dx,color);
        
        % ==================================================
        % Pose definition, based on kinematic or dynamic model
        sKinematicModel(p3dx);
        sInvKinematicModel(p3dx,dXr);
        sDynamicModel(p3dx);
        
        % ==================================================
        % Robot functions
        % Communication
        rConnect(p3dx);
        rDisconnect(p3dx);
        rSetPose(p3dx,Xo);
        % Data request
        rGetSensorData(p3dx);
        dist = rGetSonarData(p3dx);
        
        % Command
        rSendControlSignals(p3dx);
        
        % ==================================================
      
    end
end