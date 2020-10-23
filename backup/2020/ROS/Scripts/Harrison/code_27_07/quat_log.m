% This function applies the quaternion natural logarithm
function [result] = quat_log(q)
    result = q.arg() * [0, q.axis_of_rotation()];
    result = MyQuaternion(result(1), result(2), result(3), result(4));
end

