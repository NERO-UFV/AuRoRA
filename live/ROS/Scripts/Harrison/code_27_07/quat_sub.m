% This function applies the equivalent of subtraction with quaternions
function [r] = quat_sub(q, p)
    r = quat_mul(p.conj(), q.quaternion());
    r = MyQuaternion(r(1), r(2), r(3), r(4));
end

