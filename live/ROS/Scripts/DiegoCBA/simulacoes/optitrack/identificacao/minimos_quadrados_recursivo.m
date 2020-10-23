%%
%% MATLAB
%Parametros
%Codigo origem : D:\Doutorado\Codigos\controladorPD-cvdrone-master - DIF DKF - LELIS\build\vs2010\Resultados
clc, clear, close all;

% % Carrega o ultimo arquivo consolidado
fileName = strcat('Diego_identificacao_consolidado_Data_20200814T180834);   
load(fileName,'t_hist','u_w_d_hist','u_w_hist','u_p_hist','w_p_hist');

%%
t = 0;
k=1;

parametros{k} = zeros(6,1);
G1{k} = zeros(1,3);
G2{k} = zeros(1,3);

u(1,k) = 0;
u(2,k) = 0;

tempo(k) = t;

Yfp1 = 0;
Yfp2 = 0;
      
Tfp1 = G1{1};
Tfp2 = G2{1};

Theta1 = classRLS(3);
    Theta1.inicializa(Yfp1(1),G1{1});
Theta2 = classRLS(3);
    Theta2.inicializa(Yfp2(1),G2{1});

Yfp1 = vertcat(u(1,1));
Yfp2 = vertcat(u(2,1));
    
%%      
for k=2:size(u_w_d_hist,2)-1 
    
    u(1,k) = u_w_d_hist(1,k);
    u(2,k) = u_w_d_hist(2,k);  
   
   [parametros{k}, G1{k}, G2{k}] = calculo_parametros([u_p_hist(1,k); w_p_hist(1,k)],[u_w_hist(1,k); u_w_hist(2,k)], [u_w_d_hist(1,k); u_w_d_hist(2,k)]);
   
   Theta1.atualiza(u(1,k),G1{k});    
   Theta2.atualiza(u(2,k),G2{k});
     
   Yfp1 = vertcat(Yfp1, u(1,k));
   Yfp2 = vertcat(Yfp2, u(2,k));
   Tfp1 = vertcat(Tfp1, G1{k});    
   Tfp2 = vertcat(Tfp2, G2{k});       
end

%%
theta = zeros(6,1);
parametrosk = zeros(6,1);
%%
theta1 = inv(Tfp1'*Tfp1)*Tfp1'*Yfp1;
theta_1 = theta1(1);
theta_3 = theta1(2);
theta_4 = theta1(3);
%
theta2 = inv(Tfp2'*Tfp2)*Tfp2'*Yfp2;
theta_2 = theta2(1);
theta_5 = theta2(2);
theta_6 = theta2(3);

disp(['\theta_1=' num2str(theta_1) ' \theta_2=' num2str(theta_2)]);
disp(['\theta_3=' num2str(theta_3) ' \theta_4=' num2str(theta_4)]);
disp(['\theta_5=' num2str(theta_5) ' \theta_6=' num2str(theta_6)]);
%%
figure,
plot(Theta1.T(1,:)), hold on
plot(Theta1.T(2,:)),
plot(Theta1.T(3,:)), 
legend({['\theta_1 = '  num2str(Theta1.T(1,end))], ['\theta_3 = '  num2str(Theta1.T(2,end))], ['\theta_4 = '  num2str(Theta1.T(3,end))]},'FontSize',14);
% title('Variando X')
grid on
axis tight
%
figure,
plot(Theta2.T(1,:)), hold on
plot(Theta2.T(2,:)),
plot(Theta2.T(3,:)), 
legend({['\theta_2 = '  num2str(Theta2.T(1,end))], ['\theta_5 = '  num2str(Theta2.T(2,end))],['\theta_6 = '  num2str(Theta2.T(3,end))]},'FontSize',14);
% title('Variando Y')
grid on
axis tight
%