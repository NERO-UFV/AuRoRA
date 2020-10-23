%% Script used for testing

%% Cleaning:

close all;
clear all;
clc;

% %% Testing Class MyQuaternion:
% axis_of_rotation = [1, 1, 1];
% axis_of_rotation = axis_of_rotation/sqrt(sum(axis_of_rotation().^(2)));
% ang = pi/4;
% a = cos(ang);
% b = axis_of_rotation(1) * sin(ang);
% c = axis_of_rotation(2) * sin(ang);
% d = axis_of_rotation(3) * sin(ang);
% q = MyQuaternion(a, b, c, d);
% q.quaternion()
% q.real()
% q.imag()
% q.norm()
% q.normalized()
% q.arg() * 180/pi
% q.axis_of_rotation()
% sqrt(sum(q.axis_of_rotation().^(2)))
% q.conj()
% 
% %% Testing Class MyDualQuaternion:
% axis_of_rotation = [1, 0, 1];
% axis_of_rotation = axis_of_rotation/sqrt(sum(axis_of_rotation().^(2)));
% ang = pi/3;
% a = cos(ang);
% b = axis_of_rotation(1) * sin(ang);
% c = axis_of_rotation(2) * sin(ang);
% d = axis_of_rotation(3) * sin(ang);
% p = MyQuaternion(a, b, c, d);
% q.quaternion()
% p.quaternion()
% sigma = MyDualQuaternion(q.quaternion(), p.quaternion());
% sigma.dual_quaternion()
% sigma.real_part()
% sigma.dual_part()
% sigma.conjugate_1()
% sigma.conjugate_2()
% sigma.conjugate_3()
% 
% %% Testing functions:
% 
% % turn_int_quaternion:
% n = [1, 0, 1];
% ang = 60;
% q = turn_int_quaternion(ang, n);
% q.quaternion()
% 
% % quat_mul:
% q = MyQuaternion(1, 2, 3, 4);
% p = MyQuaternion(4, 3, 2, 1);
% q.quaternion()
% p.quaternion()
% g = quat_mul(q.quaternion(), p.quaternion());
% g
% 
% % quat_add:
% r = quat_add(q, p);
% r.quaternion()
% 
% % quat_sub:
% r = quat_sub(q, p);
% r.quaternion()
% 
% % quat_log:
% n = [1, 1, 1];
% ang = 45;
% p = turn_int_quaternion(ang, n);
% r = quat_log(p);
% r.quaternion()
% 
% % quat_exp:
% q = quat_exp(r);
% t = quat_exp(p);
% p.quaternion()
% q.quaternion()
% t.quaternion()
% 
% % dual_quat_mul:
% theta = 73;
% n = [6, 7, 5];
% rot = turn_int_quaternion(theta, n);
% translation = MyQuaternion(0, 10, 5, 13);
% rot.quaternion()
% translation.quaternion()
% sigma = rot_trans2dual_quat(rot, translation);
% sigma.dual_quaternion()
% sigma.magnitude()
% 
% 
% real_part_1 = [1, 0, 1, 0] * (sqrt(2)/2);
% dual_part_1 = [1, 1, 1, 1];
% real_part_2 = [0.5, 0, 0.5, 0] * sqrt(2);
% dual_part_2 = [1, 0.5, 1, 0.5];
% dual_quat_1 = MyDualQuaternion(real_part_1, dual_part_1);
% dual_quat_2 = MyDualQuaternion(real_part_2, dual_part_2);
% dual_quat_1.dual_quaternion()
% dual_quat_2.dual_quaternion()
% sigma = dual_quat_mul(dual_quat_1, dual_quat_2);
% sigma.dual_quaternion()
% sigma.magnitude()
% 
% % rot_trans2dual_quart:
% n = [1, 1, 1];
% theta = 120/2;
% rotation = turn_int_quaternion(theta, n);
% trans = [0, 0, 2];
% translation = MyQuaternion(0, trans(1), trans(2), trans(3));
% rotation.quaternion()
% translation.quaternion()
% sigma = rot_trans2dual_quat(rotation, translation);
% sigma.dual_quaternion()
% [r, t] = dual_quat2rot_trans(sigma);
% r.quaternion()
% t.quaternion()
% point = [-3, 2, -1];
% sigma = point_3d2dual_quaternion(point);
% sigma.dual_quaternion()
% 
% % rotation_translation:
% n = [1, 1, 1];
% theta = 120/2;
% rotation = turn_int_quaternion(theta, n);
% point = [3, 0, 0];
% trans = [0, 0, 2];
% result = rotation_translation(point, rotation, trans, true);
% fprintf('\n');
% n = [0, 0, 1];
% theta = -90/2;
% rotation = turn_int_quaternion(theta, n);
% point = [1, 0, 0];
% trans = [0, 0, 0];
% result = rotation_translation(point, rotation, trans, true);
% 
% % rotation:
% q = [0, 1, 0, 0];
% n = [0, 0, 1];
% ang = pi/2;
% s = quat_rotation(q, n, ang);
% s.quaternion()
% n = [1, 0, 0];
% s = quat_rotation(s.quaternion(), n, ang);
% s.quaternion()

% Pre-testing:
T = 30;
w = (2 * pi)/T;
sample = 0.03;
n = [0:sample:w*180];
axis = [0, 0, 1];
psi_measured = MyQuaternion(1, 0, 0, 0);
psi_current = MyQuaternion(1, 0, 0, 0);
psi_ponto = turn_int_quaternion(w, axis, true);
save_psi_next = [];
save_psi_next2 = [];
save_psi_ponto = [];
k1 = 0.1;
k2 = 0.1;
for k = 1:length(n)
    psi_next = path_maker(psi_current, psi_ponto, sample);
    save_psi_next = [save_psi_next; psi_next.quaternion];
    aux = psi_next.quaternion;
    psi_current = MyQuaternion(aux(1), aux(2), aux(3), aux(4));
    aux_aux = quat_add(psi_ponto, psi_current);
    save_psi_next2 = [save_psi_next2; aux_aux.quaternion];
%     save_psi_ponto = [save_psi_ponto; psi_ponto.quaternion];
%     psi_ponto = controller(psi_current, psi_measured, psi_ponto, k1, k2);
end


hold on;
grid on;
plot(n,save_psi_next(:, 1));
plot(n,save_psi_next(:, 2));
plot(n,save_psi_next(:, 3));
plot(n,save_psi_next(:, 4));
legend('w', 'i', 'j', 'k');
title('Desired Quaternions')
figure,
hold on;
grid on;
plot(n,save_psi_next2(:, 1));
plot(n,save_psi_next2(:, 2));
plot(n,save_psi_next2(:, 3));
plot(n,save_psi_next2(:, 4));
legend('w', 'i', 'j', 'k');
title('Desired Quaternions')