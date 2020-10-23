%% Script used for others tests

%% Cleaning:

close all;
clear all;
clc;

%% Testing Experiments:

% Creating Time:
T = 0.04;
total_time = 180;

% Defining initial orientation:
qd = MyQuaternion(1, 0, 0, 0);

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

    % Inside the loop:
    
    % Get q:
    % function_to_get_q
    q = MyQuaternion(1, 0, 0, 0);

    % Calculating the error:
    q_til = quat_sub(qd, q);
    % q_til_saved(k+1, :) = q_til.quaternion();

    % Computing the next desired velocity
    qd = quat_add(qd, aux);
    % qd_saved(k+1, :) = qd.quaternion();
    
    % Computing the control action:
    q_dot_aux = quat_log(q_til);
    q_dot_aux = q_dot_aux.quaternion();
    q_dot_aux = q_dot_aux * K1 * K2;
    q_dot_aux = MyQuaternion(q_dot_aux(1), q_dot_aux(2), q_dot_aux(3), q_dot_aux(4));
    q_dot = quat_add(qd_dot, q_dot_aux);
    control_signal = q_dot.arg();
    % control_signal_saved(k+1) = control_signal;