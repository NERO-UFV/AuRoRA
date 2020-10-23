clear; close all; clc;

load('Diego_identificacao7.5_Data_20200707T120212.mat')
t_hist_total = t_hist;
u_p_hist_total = u_p_hist;
u_w_d_hist_total = u_w_d_hist;
u_w_hist_total = u_w_hist;
w_p_hist_total = w_p_hist;

load('Diego_identificacao7.5_Data_20200707T120725.mat')
t_hist_total = [t_hist_total t_hist];
u_p_hist_total = [u_p_hist_total u_p_hist];
u_w_d_hist_total = [u_w_d_hist_total u_w_d_hist];
u_w_hist_total = [u_w_hist_total u_w_hist];
w_p_hist_total = [w_p_hist_total w_p_hist];

load('Diego_identificacao9_Data_20200706T155442.mat')
t_hist_total = [t_hist_total t_hist];
u_p_hist_total = [u_p_hist_total u_p_hist];
u_w_d_hist_total = [u_w_d_hist_total u_w_d_hist];
u_w_hist_total = [u_w_hist_total u_w_hist];
w_p_hist_total = [w_p_hist_total w_p_hist];

t_hist = t_hist_total;
u_p_hist = u_p_hist_total;
u_w_d_hist = u_w_d_hist_total;
u_w_hist = u_w_hist_total;
w_p_hist = w_p_hist_total;

filename = ['Diego_identificacao_consolidado' '_Data_' datestr(now,30) '.mat'];
fullname = [filename];
save(fullname,'t_hist','u_w_d_hist','u_w_hist','u_p_hist','w_p_hist')