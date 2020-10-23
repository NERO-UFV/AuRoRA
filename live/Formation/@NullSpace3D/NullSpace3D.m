%    ***************************************************************
%    Course    : Master Science                              
%    Developer : Mauro Sergio Mafra Moreira                     
%    Date      : 06/02/2019 
%    Revision  : R00                               
%                                                                 
%    History   : R00 - Inicial Emission
%
%    ***************************************************************

% Description:

classdef NullSpace3D < handle
% Establish task priority on Robots Group Formation 
  
    properties
        pPos   % Robots pose
        pPar   % Robots parameters
        pSC    % Control signal
        
        IAE    % IAE  Formation Score
        IAEt   % IAE  Formation Score
        ITAE   % ITAE Formation Score        
        ISE    % ISE  Formation Score  
        
        IAEv   % IAEv Variables Formation Score
        ISEv   % ISEv Variables Formation Score
        
        nFErr  % Normalized Formation Error
                
        WFEc   % Weighted Formation Error Coeficient
        
        SampleTime   % Sample time for simulation
        
    end
    
    methods
        
        function obj = NullSpace3D
            mInit(obj);                 % Initialize variables
        end
        % Formation Control Type        
        mFormationControl(obj);         % Formation Classical Controller 
        mPositionPriorityControl(obj);  % NSB Position  Priority Controller
        mFormationPriorityControl(obj); % NSB Formation Priority Controller
        
        
        % Auxliar Controller for Test
        mAuxPriorityControl(obj);               % Auxiliar Controller for Test
        mAuxFormationPriorityControl(obj);      % Auxiliar Controller for Test
        mAuxPositionPriorityControl(obj);       % Auxiliar Controller for Test
                

        mDirTrans(obj);                 % Direct Transformation                
        mInvTrans(obj);                 % Inverse Transformation

        % Error and Performace
        mFormationPerformace(obj,sampleTime,time,index); % NSB Formation Performace Scores
        mFormationWeightedError(obj);                    % Get Wighted Formation and Position Error
        
        
        mFormationError(obj);           % Get Formation Error
        mSetPose(obj,r1,r2);            % Set    
        
        mDisplayFormationPerformace(obj,iTitle);
        
        % Trajectory
        mTrajectoryLemniscata(obj,a,b,w,ta);
        mTrajectoryEllipse(obj,a,b,w,ta); 
                        
        % Dynamic Compensator
        robot = mDynamicCompensator(robot,cGains);
                
    end
end