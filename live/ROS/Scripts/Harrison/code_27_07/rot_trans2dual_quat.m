% This function transforms a rotation followed by a translation, all of them
% represented by quaternions, into a single dual-quaternion containing all 
% these informations
function [sigma] = rot_trans2dual_quat(r, t)
    dual_part = quat_mul(t.quaternion(), r.quaternion())/2;
    sigma = MyDualQuaternion(r.quaternion(), dual_part);
end

