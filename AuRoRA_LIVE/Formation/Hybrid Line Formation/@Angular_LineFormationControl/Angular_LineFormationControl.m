
classdef Angular_LineFormationControl < handle
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
% .................................................................
  
    properties
        pPos   % Robots pose
        pPar   % Robots parameters
        pSC    % Control signal
    end
    
    methods
        
        function obj = Angular_LineFormationControl
            mInit(obj);                 % Initialize variables
        end
        
        mInit(obj)                                      % Initialize variables
        mDirTransAngular(obj)                           % Get formation variables
        mInvTransAngular(obj)                           % Get robots position
        
        cAngular_LineFormationControl_NSB(obj,type); 
        
    end
end