% This function applies the desired controller
function [q_ponto] = controller(q_desired, q_measured, q_desired_rate, k1, k2)
    aux1 = quat_sub(q_desired, q_measured);
    aux2 = quat_log(aux1);
    aux3 = k1 * k2 * aux2.quaternion;
    aux4 = MyQuaternion(aux3(1), aux3(2), aux3(3), aux3(4));
    aux5 = quat_exp(aux4);
    aux6 = aux5.quaternion();
    aux7 = q_desired_rate.quaternion();
    aux8 = quat_mul(aux7, aux6);
    q_ponto = MyQuaternion(aux8(1), aux8(2), aux8(3), aux8(4));
end

