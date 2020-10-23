classdef Obstacle < handle
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
        function obstacle = Obstacle(ID)
            if nargin < 1
                ID = 1;
            end
            
            obstacle.pID = ID;
            
            % Dynamic Model Parameters
            % Parameters
            obstacle.pPar.g = 9.8;    % [kg.m/s^2] Gravitational acceleration
            obstacle.pPar.r = 0.100;  % Radius from central point
            obstacle.pPar.Ts = 1/30;
            obstacle.pPos.X = zeros(12,1);
            obstacle.pPos.Xa = zeros(12,1);
            obstacle.pPos.Xd = zeros(12,1);
        end
        
        % ==================================================
        % ArDrone 3D Image
        mCADCreate(obstacle);
        mCADPlot(obstacle);
       
    end
end