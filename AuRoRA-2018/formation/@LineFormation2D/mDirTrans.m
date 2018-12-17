function mDirTrans(obj)
% Direct transformation
% Calculate formation variables
%  + -------------+      +-------------------+
%  | [x1 y1 x2 y2]| ===> |[xf yf rhof alfaf] |
%  |  Formation   |      |     Formation     |
%  |   Pose(X)    |      |    variables(Q)   |
%  +--------------+      +-------------------+

% Vetor de posição dos robôs obj.pPos.X = [x1 y1 z1 rho1 theta1 psi1 x2 y2 z2 rho2 theta2 psi2]
% (não lembro porque aumentei o vetor de posição, mas deve ter tido um motivo...)

%%     Case formation reference == middle
if obj.pPar.Type(1) == 'c'
    % Transformação direta [xf yf rho alfa]
    obj.pPos.Q(1) = (obj.pPos.X(1) + obj.pPos.X(7))/2;
    obj.pPos.Q(2) = (obj.pPos.X(2) + obj.pPos.X(8))/2;
    obj.pPos.Q(3) = sqrt((obj.pPos.X(7) - obj.pPos.X(1))^2 + (obj.pPos.X(8) - obj.pPos.X(2))^2);
    obj.pPos.Q(4) = atan2((obj.pPos.X(8) - obj.pPos.X(2)),(obj.pPos.X(7) - obj.pPos.X(1)));
    
    
    %%    Case formation reference == Robot 1
elseif obj.pPar.Type(1) == 'r'
    % Transformação direta [xf yf rho alfa]
    obj.pPos.Q(1) = obj.pPos.X(1);
    obj.pPos.Q(2) = obj.pPos.X(2);
    obj.pPos.Q(3) = sqrt((obj.pPos.X(7) - obj.pPos.X(1))^2 + (obj.pPos.X(8) - obj.pPos.X(2))^2);
    obj.pPos.Q(4) = atan2((obj.pPos.X(8) - obj.pPos.X(2)),(obj.pPos.X(7) - obj.pPos.X(1)));
    %%
else
    disp('Formation type not found.');
end