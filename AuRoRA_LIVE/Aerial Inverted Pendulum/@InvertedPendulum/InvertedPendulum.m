classdef InvertedPendulum < handle
    % InvertedPendulum 
    % Creates a pendulum mounted over some platform.
    properties
        % Properties or Parameters
        pCAD  % Inverted Pendulum 3D Model
        pPar  % Parameters for Dynamic Model

        % Control variables
        pPos   % Posture
        pFlag  % Flags tag
         
    end
    methods
         function pendulum = InvertedPendulum()
            iParameters(pendulum);
            iControlVariables(pendulum);
            
            mCADmake(pendulum);
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

