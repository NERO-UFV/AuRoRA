function [x_pred R] = predice(data,R,Q)
%% Preditor by Dennis Ecuatoriano loco!
% data = dados para filtrar em colunas
% R - variavel de ajuste
% Q - variavel de ajuste
if nargin < 2
    R = 2;
    Q = .2;
end
amostras = size(data,2);
z = data;

%% Inicializa variaveis
%R Ajustable para o grau de amortecimento

indice = 1:amostras;
X_chap = 1:amostras;  %% Inicializa X chapeu
X_chap_min = 1:amostras;  %% Inicializa X chapeu minimo
Pmin1 = 0;
%%
%%%%

K = Pmin1/(Pmin1+R);


X_chap(1) = data(1);   %% Aplicando primeiro valor
	
P = (1-K)*Pmin1;
Pmin = P + Q;

X_chap_min(1) = X_chap(1);

for n = 2:amostras;
   K = Pmin/(Pmin+R);
   X_chap(n) = X_chap_min(n-1) + K*(z(n) - X_chap_min(n-1));
   P = (1-K)*Pmin;
   Pmin = P + Q;
   X_chap_min(n) = X_chap(n);
end;

x_pred = X_chap;

 %figure
 %plot(indice,data,'b-');
 %hold on;
 %plot(indice,X_chap,'g-', 'linewidth', 2)
 %legend('Original','Predito')
 %title(['Preditor de Kalman (ordem 1); R = ',num2str(R)])
end