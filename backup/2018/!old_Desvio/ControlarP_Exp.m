function [Ur,cte] = ControlarP_Exp(Xtil,alfa,a,Xd)

% sat = 0.3;
% gain = 2.5;

sat = 0.2;
gain = 2.5;

cte = [sat gain 0 0];

K = [ cos(alfa) -a*sin(alfa); sin(alfa) +a*cos(alfa) ];

F1 = ( (Xtil(1:2)>=0) - (Xtil(1:2)<0) ) .* sat.*(1-exp(-gain.*abs(Xtil(1:2))));

Ur = K \ (Xd + F1);