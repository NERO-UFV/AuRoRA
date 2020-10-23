% This function applies the dual-quaternion multiplication
function [sigma] = dual_quat_mul(sig1, sig2)
    aux1 = quat_mul(sig1.real_part(), sig2.real_part());
    aux2 = quat_mul(sig1.real_part(), sig2.dual_part()) + quat_mul(sig1.dual_part(), sig2.real_part());
    sigma = MyDualQuaternion(aux1, aux2);
end

