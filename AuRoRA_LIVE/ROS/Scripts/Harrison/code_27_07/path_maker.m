% This function makes the desired path
function [q_next] = path_maker(q_current, q_rate, sample_rate)
    aux1 = quat_log(q_rate);
    aux2 = sample_rate * aux1.quaternion;
    aux3 = MyQuaternion(aux2(1), aux2(2), aux2(3), aux2(4));
    aux4 = quat_exp(aux3);
    aux5 = aux4.quaternion();
    aux6 = q_current.quaternion();
    aux7 = quat_mul(aux6, aux5);
    q_next = MyQuaternion(aux7(1), aux7(2), aux7(3), aux7(4));
end

