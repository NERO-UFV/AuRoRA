% Calculate kinematic control velocity
function mFormationControl_NullSpace(obj,type)
% Direct transformation
obj.mDirTrans;                   % get formation variables

% Formation pose error
obj.mFormationError;

% Inverse Jacobian Matrix  {Formation vector: [xf yf zf rhof alphaf betaf]}

x1 = obj.pPos.X(1);
y1 = obj.pPos.X(2);
z1 = obj.pPos.X(3);

x2 = obj.pPos.X(4);
y2 = obj.pPos.X(5);
z2 = obj.pPos.X(6);

J = [                                                                                                                                                                    1,                                                                                                                                                                    0,                                                                                                                                               0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                0;
    0,                                                                                                                                                                    1,                                                                                                                                               0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                0;
    0,                                                                                                                                                                    0,                                                                                                                                               1,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                0;
    (x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                                            (y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                       (z1 - z2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                                          -(x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                                          -(y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                       -(z1 - z2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
    -(y1^2 - 2*y1*y2 + y2^2 + z1^2 - 2*z1*z2 + z2^2)/((1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)),                      ((2*y1 - 2*y2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), ((2*z1 - 2*z2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), (y1^2 - 2*y1*y2 + y2^2 + z1^2 - 2*z1*z2 + z2^2)/((1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)),                    -((2*y1 - 2*y2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), -((2*z1 - 2*z2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2));
    ((2*x1 - 2*x2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), -(x1^2 - 2*x1*x2 + x2^2 + z1^2 - 2*z1*z2 + z2^2)/((1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), ((2*z1 - 2*z2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)),                    -((2*x1 - 2*x2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), (x1^2 - 2*x1*x2 + x2^2 + z1^2 - 2*z1*z2 + z2^2)/((1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), -((2*z1 - 2*z2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2))];

%  Inverse Jacobian Matrix  {Formation vector: [xf yf zf rhof alphaf betaf]}

I = eye(6);

% Position
JP = J(1:3,1:6); % Position
JPinv = JP'*inv(JP*JP'); % CERTO 6x3
% Jpinv = pinv(JP)*JP  % ERRADO 6x6

JPinv;
(eye(6)-JPinv*JP);

% Shape
JS = J(4:6,1:6); % Form
JSinv = JS'*inv(JS*JS'); % CERTO 6x3
% % % % % Jsinv = pinv(Js)*Js  % ERRADO 6x6

% JSinv
% JSinv*JS

% (I - JSinv*JS)
% sadfasd = I - JPinv*JP
% inv(JS*JS')

%% Robots velocities

if type == 'S'
    % Formation Control
%     obj.pPar.K1 = diag([   10.0   10.0   10.0   1     2     2    ]);             % kinematic control gain  - controls amplitude
%     obj.pPar.K2 = diag([   0.1    0.1    0.1    0.1   0.1   0.1  ]);     % kinematic control gain - control saturation
%     
    % Form
    obj.pPos.dXr = JSinv*(obj.pPos.dQd(4:6) + obj.pPar.K1(4:6,4:6)*tanh(obj.pPar.K2(4:6,4:6)*obj.pPos.Qtil(4:6)))...
        + (I - JSinv*JS)*(JPinv*(obj.pPos.dQd(1:3) + obj.pPar.K1(1:3,1:3)*tanh(obj.pPar.K2(1:3,1:3)*obj.pPos.Qtil(1:3))));
    

%     JSinv*(obj.pPos.dQd(4:6) + obj.pPar.K1(4:6,4:6)*tanh(obj.pPar.K2(4:6,4:6)*obj.pPos.Qtil(4:6)))
%   
%     (eye(6)-JSinv*JS)*(JPinv*(obj.pPos.dQd(1:3) + obj.pPar.K1(1:3,1:3)*tanh(obj.pPar.K2(1:3,1:3)*obj.pPos.Qtil(1:3))))
   
    
    
else
    % Formation Control
%     obj.pPar.K1 = diag([   20.0  20.0   20.0    1     2     2     ]);     % kinematic control gain  - controls amplitude
%     obj.pPar.K2 = diag([   0.1    0.1    0.1    0.1   0.1   0.1   ]);     % kinematic control gain - control saturation
    
    % Position
    obj.pPos.dXr = JPinv*(obj.pPos.dQd(1:3) + obj.pPar.K1(1:3,1:3)*tanh(obj.pPar.K2(1:3,1:3)*obj.pPos.Qtil(1:3)))...
        + (I - JPinv*JP)*(JSinv*(obj.pPos.dQd(4:6) + obj.pPar.K1(4:6,4:6)*tanh(obj.pPar.K2(4:6,4:6)*obj.pPos.Qtil(4:6))));
    
      
%       JPinv*(obj.pPos.dQd(1:3) + obj.pPar.K1(1:3,1:3)*tanh(obj.pPar.K2(1:3,1:3)*obj.pPos.Qtil(1:3)))
% 
%       (eye(6)-JPinv*JP)*(JSinv*(obj.pPos.dQd(4:6) + obj.pPar.K1(4:6,4:6)*tanh(obj.pPar.K2(4:6,4:6)*obj.pPos.Qtil(4:6))))


end

obj.pPos.Xr = obj.pPos.Xr + obj.pPos.dXr*obj.pPar.Ts;

end
