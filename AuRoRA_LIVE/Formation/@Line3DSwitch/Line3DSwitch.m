
classdef Line3DSwitch < handle
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
%                        .°           
%             {Rho_f}  .°    
%                    .°       
%                  .°   {Beta_f}
%      _______   .°  )  
%     _ |   | _ º'º'º'º'º'º'º'º'º'º'º    
%    | ||___|| |  Pioneer(x1,y1,z1) = (xf,yf,zf)
%     -       -
% 
%  Ps.: The formation class can be extended to UAV-UAV without need of modifications
%  Ps2.: alpha range: [-pi,pi]   
%        beta  range: [0,pi/2]  
% .................................................................
  
    properties
        pPos   % Robots pose
        pPar   % Robots parameters
        pSC    % Control signal
    end
    
    methods
        
        function obj = Line3DSwitch
            mInit(obj);                 % Initialize variables
        end
        
        mInit(obj)                      % Initialize variables
        mFormationControl(obj);         % Formation Controller
        mTransDir(obj,var)                  % Get formation variables
        mTransInv(obj,var)                  % Get robots position
    end
end