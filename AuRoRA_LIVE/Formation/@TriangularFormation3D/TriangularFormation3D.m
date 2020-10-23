
classdef TriangularFormation3D < handle
% Establish a Triangular Formation in 3D
% 
% FORMATION DESIGN ................................................
% [Top view] 
%           
% [Front view]  
% 

    properties
        pPos   % Formation pose
        pPar   % Formation parameters
        pSC    % Control signal
    end
    
    methods
        
        function obj = TriangularFormation3D
            mInit_TF3D(obj);                 % Initialize variables
        end
        
        mInit_TF3D(obj)                % Initialize variables
        mIdentSeq_TF3D(obj)            % Sequence Identification
        mDirTrans_TF3D(obj)            % Get formation variables
        mInvTrans_TF3D(obj)            % Get robots position
        mFormationError_TF3D(obj)           % Get formation error
        mTrajectoryPlanner_TF3D(obj,t)
%         mTrajectoryPlanner_TF3D_mexido(obj,t)
        
        % Controllers
        cNoPriority_TF3D(obj)
        cNullSpaceBased_TF3D(obj,priority)

    end
end