
function mSetPose(obj,r1,r2)

% Set current position
obj.pPos.X = [r1.pPos.X(1:3); r2.pPos.X(1:3)];

end