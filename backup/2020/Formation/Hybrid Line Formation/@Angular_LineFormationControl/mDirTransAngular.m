% Direct transformation
% Calculate formation variables using robots pose
%
%  + ------------------+      +--------------------------+
%  | x1 y1 z1 x2 y2 z2 | ===> |xF yF zF rhof alpha beta  |
%  | Formation pose(X) |      |  Formation variables(Q)  |
%  +-------------------+      +--------------------------+

function mDirTransAngular(obj)

    x1 = obj.pPos.X(1,1);   
    y1 = obj.pPos.X(2,1);
    z1 = obj.pPos.X(3,1);
    x2 = obj.pPos.X(4,1);
    y2 = obj.pPos.X(5,1);
    z2 = obj.pPos.X(6,1);
    
    obj.pPos.Q(1) = x1;                                             % xF
    obj.pPos.Q(2) = y1;                                             % yF
    obj.pPos.Q(3) = z1;                                             % zF
    obj.pPos.Q(4) = sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2);        % rho
    obj.pPos.Q(5) = asin((x2-x1)/obj.pPos.Q(4)) ;                   % alpha
    obj.pPos.Q(6) = asin((y2-y1)/obj.pPos.Q(4));                    % beta
    
end