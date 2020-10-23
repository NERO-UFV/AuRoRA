classdef Cable < handle
    % In a methods block, set the method attributes
    % and add the function signature
    properties
        % Properties or Parameters
        pCAD   % ArDrone 3D image
        pPar   % Parameters Dynamic Model
        pID    % Identification

        % Control variables
        pPos   % Posture
        pFlag  % Flags

        % Navigation Data and Communication
        pData % Flight Data
        pCom  % Communication
    end
    methods
        function cable = Cable(ID)
            if nargin < 1
                ID = 1;
            end
			cable.pID = ID;
			
            % Cable Parameters
            %% ========================================================================
            % Massa e Momento de Inérica
            cable.pPar.g = 9.8; % Aceleração da gravidade
            cable.pPar.l = 1;
            cable.pCAD.cor = 'k';
            cable.pPar.Ts = 1/30;
            
            %% Cabos
            cable.pPos.X = [0 0 0 0]; % Postura Atual - Alpha, Beta, Tração e Comprimento do cabo
            cable.pPos.dX = [0 0 0 0]; % Primeira derivada - Postura Atual - Alpha, Beta, Tração e Comprimento do cabo
            cable.pPos.ddX = [0 0 0 0]; % Segunda derivada - Postura Atual - Alpha, Beta, Tração e Comprimento do cabo
            cable.pPos.Xa = [0 0 0 0]; % Postura Anterior
        end
        
        mInit(cable);
        
        % Funções Relativas ao cable
        mTractionForce(cable,load,drone);
        mCableStretchingEff(cable);
        
        % ==================================================
        % Cable 3D Image
		mCADCreate(cable,load,drone);
        mCADPlot(cable,load,drone);
    end
end