classdef Load < handle
    properties
        % Properties or Parameters
        pCAD   % ArDrone 3D image
        pPar   % Parameters Dynamic Model
        pID    % Identification

        % Control variables
        pPos   % Posture
        pSC    % Signals
        pFlag  % Flags

        % Navigation Data and Communication
        pData % Flight Data
        pCom  % Communication
    end
    methods
        function load = Load(ID)
            if nargin < 1
                ID = 1;
            end
            load.pID = ID;
            
            load.pFlag.Connected = 0;
            iControlVariables(load);
            iParameters(load);
        end
		% ==================================================
        iControlVariables(load);
        iParameters(load);
		
        % ==================================================
		sLoadDynamicModel(load,drone);
		sCalculateAngles(load,drone);

        % ==================================================
        % Load 3D Image
        mCADCreate(load,drone);
        mCADPlot(load,visible);
    end
end