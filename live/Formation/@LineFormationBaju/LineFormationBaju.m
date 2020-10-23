classdef LineFormationBaju < handle
% Establish a Line Formation in 3D
% (The center of formation is defined at robot 1)
% 
% FORMATION DESIGN ................................................
% [Superior view] 
%           
%                      Drone (x2,y2,z2)
%                      O  O
%                       \/    
%                    .° /\   
%                  .°  O  O
%                .°       
%              .°
%       __   .°  ) {Alpha_f}
%      /  \  º'º'º'º'º'º'º'º'º'º'º     
%    ||    || Pioneer(x1,y1,z1) = (xf,yf,zf)
%      `--´ 
% 
% [Frontal view]  
% 
%          
%                          Drone (x2,y2,z2)
%                                 _   
%                          %%%%%-(°)-%%%%%    
%              |         .°           
%              |       .°    
%              |     .° {r_f}      
%     {delta_f}|__ .°   
%      _______ | .°     {Rho_f}
%     _ |   | _ º'º'º'º'º'º'º'º'º'º'º    
%    | ||___|| |  Pioneer(x1,y1,z1) = (xf,yf,zf)
%     -       -
% 
%  Ps.: The formation class can be extended to UAV-UAV without need of modifications
%  Ps2.: alpha range: [-pi,pi]   
%        beta  range: [0,pi/2]  
% .................................................................     
    properties
        pPos    % Robots pose
        pPar    % Robots parameters
        pSC     % Control signal
        pID     % Formation ID
    end
    
    methods
        function linef = LineFormationBaju(ID)
            if nargin < 1
                ID = 1;
            end
            fInit(linef);           % Initialize variables
            linef.pID = ID;
        end
        
        fInit(linef);               % Initialize variables
        fFormationControl(linef);   % Formation Controller
        fDirTrans(linef);           % Get formation variables
        fInvTrans(linef);           % Get robots position
        fFormationError(linef);     % Formation Error
    end
end
            