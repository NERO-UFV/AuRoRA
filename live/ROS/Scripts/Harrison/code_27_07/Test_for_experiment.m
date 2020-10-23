%% Script used for others tests

%% Cleaning:

close all;
clear all;
clc;

%% Testing Experiments:

% Creating Time:
T = 0.04;
total_time = 180;
t = zeros(((total_time/T) + 1), 1);
for k = 1:(length(t) - 1)
    t(k+1) = T + t(k);
end

% Defining initial orientation:
q = MyQuaternion(1, 0, 0, 0);
q_saved = zeros((length(t) + 1), 4);
q_saved(1, :) = [1, 0, 0, 0];
qd = MyQuaternion(sqrt(2)/2, 0, 0, sqrt(2)/2);
qd_saved = zeros((length(t) + 1), 4);
qd_saved(1, :) = [sqrt(2)/2, 0, 0, sqrt(2)/2];
q_til_saved = zeros((length(t) + 1), 4);
control_signal_saved = zeros((length(t) + 1), 1);

% Defining rotation speed:
w = 6;
n = [0, 0, 1];
qd_dot = turn_int_quaternion(w, n);

% Creating the desired orientation:
aux = quat_log(qd_dot);
aux = aux.quaternion();
aux = T * aux;
aux = MyQuaternion(aux(1), aux(2), aux(3), aux(4));
aux = quat_exp(aux);
K1 = 1;
K2 = 1;
for k = 1:length(t)
% for k = 1:3
    % Calculating the error:
    q_til = quat_sub(qd, q);
    q_til_saved(k+1, :) = q_til.quaternion();
    
    % Computing the next desired velocity
    qd = quat_add(qd, aux);
    qd_saved(k+1, :) = qd.quaternion();
    
    % Computing the control action:
    q_dot_aux = quat_log(q_til);
    q_dot_aux = q_dot_aux.quaternion();
    q_dot_aux = q_dot_aux * K1 * K2;
    q_dot_aux = MyQuaternion(q_dot_aux(1), q_dot_aux(2), q_dot_aux(3), q_dot_aux(4));
    q_dot = quat_add(qd_dot, q_dot_aux);
    aux_control = q_dot.arg();
    aux_control = aux_control * 180/pi;
    control_signal_saved(k+1) = aux_control;
    
    % Computing the next q:
    aux2 = quat_log(q_dot);
    aux2 = aux2.quaternion();
    aux2 = aux2 * T;
    aux2 = MyQuaternion(aux2(1), aux2(2), aux2(3), aux2(4));
    aux2 = quat_exp(aux2);
    q = quat_add(q, aux2);
    q_saved(k+1, :) = q.quaternion();
end

% Plotting:
subplot(3, 1, 1);
hold on;
plot(t, qd_saved(1:(end-1), 1));
plot(t, qd_saved(1:(end-1), 2));
plot(t, qd_saved(1:(end-1), 3));
plot(t, qd_saved(1:(end-1), 4));
grid on;
title('Desired Orientation');
legend('w', 'i', 'j', 'k');
subplot(3, 1, 2);
hold on;
plot(t, q_saved(1:(end-1), 1));
plot(t, q_saved(1:(end-1), 2));
plot(t, q_saved(1:(end-1), 3));
plot(t, q_saved(1:(end-1), 4));
grid on;
title('Simulated Orientation');
legend('w', 'i', 'j', 'k');
subplot(3, 1, 3);
plot(t, control_signal_saved(1:(end-1)));
grid on;
title('Euclidian Control Signal');
legend('º');
figure,
hold on;
plot(t, q_til_saved(1:(end-1), 1));
plot(t, q_til_saved(1:(end-1), 2));
plot(t, q_til_saved(1:(end-1), 3));
plot(t, q_til_saved(1:(end-1), 4));
grid on;
title('Simulated Error');
legend('w', 'i', 'j', 'k');


