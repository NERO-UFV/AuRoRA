classdef InvPendulum < handle
    %InvPendulum Implementa carga de pêndulo invertido para ArDrone
    %   Creates a pontual load with mass mc, attached to a weightless 
    %  rod with length r, positioned on top of the Parrot 2.0 drone's
    %  center of mass, destined to be used with model in the script 
    %  sInvPendDynamicModel.
    %   Cria carga de massa mc com haste de comprimento r,
    %  posicionada acima do centro de massa do drone Parrot 2.0, 
    %  destinada a ser utilizada com modelo em sInvPendDynamicModel.
    
    properties
        % Properties or Parameters
%         pCADp  % Inverted Pendulum 3D Model
        pCAD  % Drone 3D Model
        pPar   % Parameters for Dynamic Model
        pID    % Identification tag
        
        % Control variables
        pPos   % Posture
        pSC    % Control signals
        pFlag  % Flags tag
        
        % Navigation Data and Communication
        pData % Flight Data
        pCom  % Communication
         
    end
    methods
         function pendulum = InvPendulum(ID)
            if nargin < 1
                ID = 1;
            end
            
            pendulum.pID = ID; 
            iControlVariables(pendulum);
            iParameters(pendulum);
            mCADload(pendulum);
%             mPREplot(InvPendulum);
        end
        
        % ==================================================
        % Inverted Pendulum preview plot
%         mPREplot(pendulum);
        % ==================================================
        % Inverted Pendulum 3D Image
        % ...
        %
        % ==================================================
%         Drone 3D Image
        mCADload(pendulum);
        mCADcolor(pendulum,cor);
        mCADplot(pendulum,visible);
        mCADdel(pendulum);
        
        % ==================================================
        iControlVariables(pendulum);
        iParameters(pendulum);
        
        % ==================================================
        sInvPendDynamicModel(pendulum);
        
    end
end

