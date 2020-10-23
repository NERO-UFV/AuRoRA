function [Ur,cte] = ControlarP_Tanh(Xtil,alfa,a,Xd)

% sat = 0.3;
% gain = 2.50;

sat = 0.2;
gain = 2.0;


cte = [sat gain 0 0];

K = [ cos(alfa) -a*sin(alfa); sin(alfa) +a*cos(alfa) ];

F1 = sat .* tanh( gain * Xtil(1:2) );
        
Ur = K \ (Xd + F1);