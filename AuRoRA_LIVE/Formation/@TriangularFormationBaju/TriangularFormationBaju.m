classdef TriangularFormationBaju < handle
% Establish a Triangle Formation
% (The center of formation is defined at robot 1)
%
% FORMATION DESIGN ...........................................................
% [Frontal view]  
% 
%                        Drone1 (x1,y1,z1)
%              Z                 _
%              ^          %%%%%-(°)-%%%%%                      
%              |         /\
%              |        /  \{sf} 
%              |       /    \   Drone2 (x2,y2,z2)
%              |  {pf}/      \         _   
%              |     /        \ %%%%%-(°)-%%%%%    
%              |    /         .°           
%      {thetaf}|___/{betaf}.°    
%              |  /\    .°       
%              | /  \.° {qf}
%      _______ |/ .°                   
%     _ |   | _ º'º'º'º'º'º'º'º'º'º'º'º> X 
%    | ||___|| |  Pioneer(x,y,z) = (xf,yf,zf) 
%     -       -                              
% 
%  Ps.: The formation class can be extended to UAV-UAV-UAV without need of modifications
%  Ps2.: - phif   is the angle of rotation relative to the x axis
%        - thetaf is the angle of rotation relative to the y axis
%        - psif   is the angle of rotation relative to the z axis
%  Ps3.: betaf range: [0,pi]
%        phif range:  [-pi,pi] 
%        thetaf range:[-pi,pi]
%        psif range:  [-pi,pi]
% ............................................................................
    properties
        pPos    % Robots pose
        pPar    % Robots parameters
        pSC     % Control signal
        pID     % Formation ID
    end
    
    methods
        function trif = TriangularFormationBaju(ID)
            if nargin < 1
                ID = 1;
            end
            tInit(trif);           % Initialize variables
            trif.pID = ID;
        end
        
        tInit(trif);               % Initialize variables
        tFormationControl(trif);   % Formation Controller
        tDirTrans(trif);           % Get formation variables
        tInvTrans(trif);           % Get robots position
        tFormationError(trif);     % Formation Error
    end
end
