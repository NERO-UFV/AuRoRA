% This function applies the equivalent of addition with quaternions
function [r] = quat_add(q, p)
    r = quat_mul(q.quaternion(), p.quaternion());
    r = MyQuaternion(r(1), r(2), r(3), r(4));
end
