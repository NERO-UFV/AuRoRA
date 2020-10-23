clear all
close all
clc

X_robo = [0 0 0 pi/2]'; % posicao atual do robo
X_trailer1 = [0 0 0 pi/4]';  % posicao atual do trailer1
X_trailer2 = [0 0 0 -pi/4]';  % posicao atual do trailer1
X_trailer2_aux = [0 0 0 -pi/4]';

%%% dimensoes trailers
L0 = 0.3;
L1 = 0.455;
L10 = 0.28;
L2 = 0.455;
ret_lg = 0.415;
ret_lp = 0.285;
b = 0.2075;

x_t1 = X_robo(1) - L0*cos(X_robo(4)) - (L1)*cos(X_trailer1(4));
y_t1 = X_robo(2) - L0*sin(X_robo(4)) - (L1)*sin(X_trailer1(4));

x_t2 = x_t1 - L10*cos(X_trailer1(4)) - (L2+b)*cos(X_trailer2(4));
y_t2 = y_t1 - L10*sin(X_trailer1(4)) - (L2+b)*sin(X_trailer2(4));

x_t2_c = x_t1 - L10*cos(X_trailer1(4)) - (L2)*cos(X_trailer2_aux(4));
y_t2_c = y_t1 - L10*sin(X_trailer1(4)) - (L2)*sin(X_trailer2_aux(4));

X_trailer1([1 2]) = ([x_t1 y_t1]);
X_trailer2([1 2]) = ([x_t2 y_t2]);
X_trailer2_aux([1 2]) = ([x_t2_c y_t2_c]);  %%% centro do trailer 2


% Variaveis para plot corpo
raio = 0.15;
circ = 0:0.01:2*pi;

Corpo1 = [raio*cos(circ);raio*sin(circ)] + X_robo(1:2);

Corpo2 = [cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[-ret_lg/2 ret_lg/2 ret_lg/2 -ret_lg/2 -ret_lg/2;-ret_lp/2 -ret_lp/2 ret_lp/2 ret_lp/2 -ret_lp/2] + X_trailer1(1:2);
haste_L0 = L0*[cos(X_robo(4)) -sin(X_robo(4));sin(X_robo(4)) cos(X_robo(4))]*[-1;0] + X_robo(1:2);
haste_L1 = (L1)*[cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[1;0] + X_trailer1(1:2);

% Corpo3 = [cos(X_trailer2(4)) -sin(X_trailer2(4));sin(X_trailer2(4)) cos(X_trailer2(4))]*[-ret_lg/2 ret_lg/2 ret_lg/2 -ret_lg/2 -ret_lg/2;-ret_lp/2 -ret_lp/2 ret_lp/2 ret_lp/2 -ret_lp/2] + X_trailer2(1:2);
% haste_L10 = L10*[cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[-1;0] + X_trailer1(1:2);
% haste_L2 = (L2)*[cos(X_trailer2(4)) -sin(X_trailer2(4));sin(X_trailer2(4)) cos(X_trailer2(4))]*[1;0] + X_trailer2(1:2);

Corpo3 = [cos(X_trailer2_aux(4)) -sin(X_trailer2_aux(4));sin(X_trailer2_aux(4)) cos(X_trailer2_aux(4))]*[-ret_lg/2 ret_lg/2 ret_lg/2 -ret_lg/2 -ret_lg/2;-ret_lp/2 -ret_lp/2 ret_lp/2 ret_lp/2 -ret_lp/2] + X_trailer2_aux(1:2);
haste_L10 = L10*[cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[-1;0] + X_trailer1(1:2);
haste_L2 = (L2)*[cos(X_trailer2_aux(4)) -sin(X_trailer2_aux(4));sin(X_trailer2_aux(4)) cos(X_trailer2_aux(4))]*[1;0] + X_trailer2_aux(1:2);

plot(X_robo(1),X_robo(2))
hold on
grid on
plot(X_trailer1(1),X_trailer1(2))
plot(X_trailer2(1),X_trailer2(2),'*')
plot(Corpo1(1,:),Corpo1(2,:))
plot(Corpo2(1,:),Corpo2(2,:))
plot(Corpo3(1,:),Corpo3(2,:))
plot([X_robo(1) haste_L0(1)],[X_robo(2) haste_L0(2)])
plot([X_trailer1(1) haste_L1(1)],[X_trailer1(2) haste_L1(2)])
plot([haste_L1(1)],[haste_L1(2)],'bo')
plot([X_trailer1(1) haste_L10(1)],[X_trailer1(2) haste_L10(2)])
plot([X_trailer2_aux(1) haste_L2(1)],[X_trailer2_aux(2) haste_L2(2)])
plot([haste_L2(1)],[haste_L2(2)],'bo')
axis(2*[-1 1 -1 1])

