clear; close all; clc;

load('Diego_identificacao120_Data_20200814T175253.mat')
t_hist_total = t_hist;
u_p_hist_total = u_p_hist;
u_w_d_hist_total = u_w_d_hist;
u_w_hist_total = u_w_hist;
w_p_hist_total = w_p_hist;

load('Diego_identificacao120_Data_20200814T175546.mat')
t_hist_total = [t_hist_total t_hist];
u_p_hist_total = [u_p_hist_total u_p_hist];
u_w_d_hist_total = [u_w_d_hist_total u_w_d_hist];
u_w_hist_total = [u_w_hist_total u_w_hist];
w_p_hist_total = [w_p_hist_total w_p_hist];

load('Diego_identificacao120_Data_20200814T180141.mat')
t_hist_total = [t_hist_total t_hist];
u_p_hist_total = [u_p_hist_total u_p_hist];
u_w_d_hist_total = [u_w_d_hist_total u_w_d_hist];
u_w_hist_total = [u_w_hist_total u_w_hist];
w_p_hist_total = [w_p_hist_total w_p_hist];
% % % % 
% load('Diego_identificacao60_Data_20200814T171600.mat')
% t_hist_total = [t_hist_total t_hist];
% u_p_hist_total = [u_p_hist_total u_p_hist];
% u_w_d_hist_total = [u_w_d_hist_total u_w_d_hist];
% u_w_hist_total = [u_w_hist_total u_w_hist];
% w_p_hist_total = [w_p_hist_total w_p_hist];
% % % 
% load('Diego_identificacao60_Data_20200814T171756.mat')
% t_hist_total = [t_hist_total t_hist];
% u_p_hist_total = [u_p_hist_total u_p_hist];
% u_w_d_hist_total = [u_w_d_hist_total u_w_d_hist];
% u_w_hist_total = [u_w_hist_total u_w_hist];
% w_p_hist_total = [w_p_hist_total w_p_hist];
% % 
% load('Diego_identificacao60_Data_20200814T174009.mat')
% t_hist_total = [t_hist_total t_hist];
% u_p_hist_total = [u_p_hist_total u_p_hist];
% u_w_d_hist_total = [u_w_d_hist_total u_w_d_hist];
% u_w_hist_total = [u_w_hist_total u_w_hist];
% w_p_hist_total = [w_p_hist_total w_p_hist];
% % 
% load('Diego_identificacao60_Data_20200814T173350.mat')
% t_hist_total = [t_hist_total t_hist];
% u_p_hist_total = [u_p_hist_total u_p_hist];
% u_w_d_hist_total = [u_w_d_hist_total u_w_d_hist];
% u_w_hist_total = [u_w_hist_total u_w_hist];
% w_p_hist_total = [w_p_hist_total w_p_hist];

t_hist = t_hist_total;
u_p_hist = u_p_hist_total;
u_w_d_hist = u_w_d_hist_total;
u_w_hist = u_w_hist_total;
w_p_hist = w_p_hist_total;

filename = ['Diego_identificacao_consolidado' '_Data_' datestr(now,30) '.mat'];
fullname = [filename];
save(fullname,'t_hist','u_w_d_hist','u_w_hist','u_p_hist','w_p_hist')