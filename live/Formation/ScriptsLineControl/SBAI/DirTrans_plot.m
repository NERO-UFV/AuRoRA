% Direct transformation
% Calculate formation variables using robots pose
%
%  + ------------------+      +--------------------------+
%  | x1 y1 z1 x2 y2 z2 | ===> |xF yF zF rhof alpha beta|
%  | Formation pose(X) |      |  Formation variables(Q)  |
%  +-------------------+      +--------------------------+

function R = DirTrans_plot(X)
R = [];
for i=1:length(X)
    x1 = X(1,i);   
    y1 = X(2,i);
    z1 = X(3,i);
    x2 = X(4,i);
    y2 = X(5,i);
    z2 = X(6,i);

    Q(1) = x1;                                         % xF
    Q(2) = y1;                                         % yF
    Q(3) = z1;                                         % zF
    Q(4) = sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2);    % rho
    Q(5) = asin((x2-x1)/Q(4)) ;                        % alpha
    Q(6) = asin((y2-y1)/Q(4));                         % beta
    
    R = [R ; Q];
end
R = R';
end