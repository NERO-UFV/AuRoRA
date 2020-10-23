% This function transforms a single dual-quaternion  containing informations
% about a rotation followed by a translation into two quaternions: one
% giving the rotation information and other giving the translation
% information.
function [r, t] = dual_quat2rot_trans(sigma)
    real_part = sigma.real_part();
    sigma_conjugate_2_real_part = sigma.conjugate_2();
    aux = 2 * quat_mul(sigma.dual_part(), sigma_conjugate_2_real_part);
    t = MyQuaternion(aux(1), aux(2), aux(3), aux(4));
    r = MyQuaternion(real_part(1), real_part(2), real_part(3), real_part(4));
end

