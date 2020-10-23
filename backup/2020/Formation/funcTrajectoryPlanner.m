function [ Xd ,dXd , ddXd ] = funcTrajectoryPlanner(TimeVars,TrajConst,type)

% This function calculates the position, velocity and acceleration values 
% for a given selected trajectory.

%   Trajectory Types

%       (1) 'Lemniscate - Horizontal'
%       (2) 'Lemniscate - Horizontal with double parameterization in time'
%       (3) 'Lemniscate - Inclined'
%       (4) 'Lemniscate - Vertical ( XZ plan | Y = cte )'
%       (5) 'Lemniscate - Vertical ( YZ plan | X = cte )'

%% Applying variable values
% Time Variables
tinc = TimeVars(1);
t = TimeVars(2);

% Trajectory constraints
if nargin < 2
    rX = 1;      % X-axis radius [m]
    rY = 1;      % Y-axis radius [m]
    rZ = 1;      % Z-axis radius [m]
    T = 30;      % Period [s]
    TF = T*2;    % Final time for two turns [s]
    w = 2*pi/T;  % Angular frequency[rad/s]
else
    rX = TrajConst(1);  % X-axis radius [m]
    rY = TrajConst(2);  % Y-axis radius [m]
    rZ = TrajConst(3);  % Z-axis radius [m]
    T = TrajConst(4);   % Period [s]
    TF = TrajConst(5);  % Final time for two turns [s]
    w = TrajConst(6);   % Angular frequency[rad/s]
end

if nargin < 3
    type = 'Lemniscate - Horizontal';
end

%% Calculating the trajectory
switch type
    
    case 'Lemniscate - Horizontal' 
        
        Xd = [ rX*sin(w*t)   ;
               rY*sin(2*w*t) ;
               rZ            ;
               0             ];
            
        dXd = [ w*rX*cos(w*t)     ;
                2*w*rY*cos(2*w*t) ;
                0                 ;
                0                 ];
            
        ddXd = [ -w^2*rX*sin(w*t)       ;
                 -(2*w)^2*rY*sin(2*w*t) ;
                 0                      ;
                 0                      ];
             
    case 'Lemniscate - Horizontal with double parameterization in time'
        
        % Auxiliary variables
        % Present values
        a = 3*(t/TF)^2 - 2*(t/TF)^3;
        tp = a*TF;
        
        % Past values
        tant = t - tinc;
        aant = 3*(tant/TF)^2 - 2*(tant/TF)^3;
        tpant = aant*TF;
        
        Xd = [ rX*sin(w*tp)   ;
               rY*sin(2*w*tp) ;
               rZ             ;
               0              ];
        
        dXdant = [ w*6*(((tant)/TF)-(tant/TF)^2)*rX*cos(w*tpant)   ;
                   2*w*6*((tant/TF)-(tant/TF)^2)*rY*cos(2*w*tpant) ;
                   0                                               ;
                   0                                               ];
        
        dXd = [  w*6*((t/TF)-(t/TF)^2)*rX*cos(w*tp)     ;
                 2*w*6*((t/TF)-(t/TF)^2)*rY*cos(2*w*tp) ;
                 0                                      ;
                 0                                      ];
        
        ddXd = [ (dXd(1) - dXdant(1))/tinc ;
                 (dXd(2) - dXdant(2))/tinc ;
                 0                         ;
                 0                         ];
             
    case 'Lemniscate - Inclined'
        
        Xd = [ rX*sin(w*t)          ;
               rY*sin(2*w*t)        ;
               rZ + 0.4*rZ*sin(w*t) ;
               pi/6*sin(w*t)        ];
            
        dXd = [ w*rX*cos(w*t)      ;
                2*w*rY*cos(2*w*t)  ;
                0.4*w*rZ*cos(w*t)  ;
                w*pi/6*cos(w*t)    ];
            
        ddXd = [ -w^2*rX*sin(w*t)       ;
                 -(2*w)^2*rY*sin(2*w*t) ;
                 -0.4*w^2*rZ*sin(w*t)   ;
                 -w^2*pi/6*sin(w*t)     ];
     
    case 'Lemniscate - Vertical ( XZ plan | Y = cte )'
        
        Xd = [ rX*sin(2*w*t)         ;
               rY                    ;
               rZ + 0.4*rZ*sin(w*t)  ;
               0                     ];
            
        dXd = [ 2*w*rX*cos(2*w*t) ;
                0                 ;
                0.4*w*rZ*cos(w*t) ;
                0                 ];
            
        ddXd = [ -(2*w)^2*rX*sin(2*w*t) ;
                 0                      ;
                 -0.4*w^2*rZ*sin(w*t)   ;
                 0                      ];
                 
    case 'Lemniscate - Vertical ( YZ plan | X = cte )'
        
        Xd = [ rX                   ;
               rY*sin(2*w*t)        ;
               rZ + 0.4*rZ*sin(w*t) ;
               0                    ];
            
        dXd = [ 0                 ;
                2*w*rY*cos(2*w*t) ;
                0.4*w*rZ*cos(w*t) ;
                0                 ];
            
        ddXd = [ 0                      ;
                 -(2*w)^2*rY*sin(2*w*t) ;
                 -0.4*w^2*rZ*sin(w*t)   ;
                 0                      ];
        
end