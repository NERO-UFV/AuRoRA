% This function applies the quaternion natural exponential
function [result] = quat_exp(q)
    amplitude = exp(q.real());
    angle = sqrt(sum(q.imag().^(2)));
    result = turn_int_quaternion(angle, q.imag(), true);
    result = amplitude * result.quaternion();
    result = MyQuaternion(result(1), result(2), result(3), result(4));
end

