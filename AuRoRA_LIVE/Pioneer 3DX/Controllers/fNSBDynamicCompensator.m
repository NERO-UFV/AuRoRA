
function obj = mDynamicCompensator(obj,cGains)

   
% cgains is a 4 elements vector input with  dynamic gains:

% Case do not have input gains
if nargin < 2
%     disp('Gains not given. Using standard ones.');
    cgains = [0.75 0.75 0.12 0.035];
end


Kdin1 = diag(cgains(1:2));    % maximum value gain
Kdin2 = diag(cgains(3:4));    % saturation gain

% % Dynamic compensator gains
% Kdin1 = diag([1 .01]);
% Kdin2 = diag([1.2 0.35]);


H = [obj.pPar.theta(1)        0;
    0           obj.pPar.theta(2)];

C = [obj.pPar.theta(4)                -obj.pPar.theta(3)*obj.pSC.U(2);
    obj.pPar.theta(5)*obj.pSC.U(2)            obj.pPar.theta(6)     ];

% Kalman Filter
obj.pSC.dUd = (obj.pSC.U - obj.pSC.Ua)/0.1;  % calcula aceleração

% nu = Kdin1*tanh(Kdin2*(robo.pSC.Ur(1:2)-robo.pSC.U));
% nu = 0.1*tanh(10*robot.pSC.dUd) + Kdin1*tanh(Kdin2*(robot.pSC.Ur(1:2)-robot.pSC.U));
nu = 0.1*tanh(.1*obj.pSC.dUd) + Kdin1*tanh(Kdin2*(obj.pSC.Ur(1:2)-obj.pSC.U));
% nu = .6*tanh(robot.pSC.dUd) + Kdin1*tanh(Kdin2*(robot.pSC.Ur(1:2)-robot.pSC.U));

% Reference control Signal
obj.pSC.Ud = H*nu + C*obj.pSC.Ur(1:2);


% Save last velocity
obj.pSC.Ua = obj.pSC.Ur(1:2);

end

