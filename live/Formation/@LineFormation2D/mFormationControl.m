%% Calculate Kinematic Control Velocity
function mFormationControl(obj)

% Direct Transformation
obj.mDirTrans;                   % calculate formation variables

% Formation error
obj.pPos.Qtil = obj.pPos.Qd - obj.pPos.Q;
% Quadrant correction
if abs(obj.pPos.Qtil(4)) > pi
    if obj.pPos.Qtil(4) > 0
        obj.pPos.Qtil(4) = -2*pi + obj.pPos.Qtil(4);
    else
        obj.pPos.Qtil(4) =  2*pi + obj.pPos.Qtil(4);
    end
end

%% Inverse Jacobian

% Formation reference between robots
if obj.pPar.Type(1) == 'c'
    
    % Inverse Jacobian
    Jinv = [ 1, 0, -cos(obj.pPos.Q(4))/2, obj.pPos.Q(3)*sin(obj.pPos.Q(4))/2;
        0, 1, -sin(obj.pPos.Q(4))/2, -obj.pPos.Q(3)*cos(obj.pPos.Q(4))/2;
        1, 0, cos(obj.pPos.Q(4))/2 , -obj.pPos.Q(3)*sin(obj.pPos.Q(4))/2;
        0, 1, sin(obj.pPos.Q(4))/2 , obj.pPos.Q(3)*cos(obj.pPos.Q(4))/2];
    
    % Formation reference on robot 1
elseif obj.pPar.Type(1) =='r'    
    
    % Inverse Jacobian
    Jinv = [ 1, 0,       0,                     0;
        0, 1,       0,                     0;
        1, 0, cos(obj.pPos.Q(4)) , -obj.pPos.Q(3)*sin(obj.pPos.Q(4));
        0, 1, sin(obj.pPos.Q(4)) , obj.pPos.Q(3)*cos(obj.pPos.Q(4))];
else
    disp('Formation type not found.');   
end
%%
% Formation Velocity
obj.pPos.dQr = obj.pPos.dQd + obj.pPar.K1*tanh(obj.pPar.K2*obj.pPos.Qtil);

% Robots Velocities
obj.pPos.dXr = Jinv*obj.pPos.dQr;

end