classdef Catamara < handle
    
    properties
        
        % Properties or Parameters
        pCAD   % Catamara 3D image
        pPar   % Parameters Dynamic Model
        pID    % Identification
        
        % Control variables
        pPos   % Posture
        pSC    % Signals
        pFlag  % Flags
        pPesoVirtual % Peso virtual do catamara
        
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
        end
           
                
        
        % ==================================================
        % Catamara 3D Image
        mCADload(catamara);
        mCADcolor(catamara,cor);
        mCADplot(catamara);
        
        % ==================================================
        iControlVariables(catamara);
        iParameters(catamara);
        
        % ==================================================
        %sDynamicModel(catamara);
        

        
        
    end
end