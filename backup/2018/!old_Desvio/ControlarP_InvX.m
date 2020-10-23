function [Ur,cte] = ControlarP_InvX(Xtil,alfa,a,Xd)

% sat1 = 0.3;
% gain1 = 2.5;
% 
% sat2 = 5e-2;
% gain2 = 2.5;

sat1 = 0.2;
gain1 = 2.0;

sat2 = 5e-2;
gain2 = 2.5;

cte = [sat1 gain1 sat2 gain2];

K = [ cos(alfa) -a*sin(alfa); sin(alfa) +a*cos(alfa) ];

F1 = sat1 .* tanh( gain1 * Xtil(1:2) );

% F2 = sat2*tanh( .001./ (Xtil(1:2))).*tanh( 2.5*Xtil(1:2).^2 );
F2 = sat2./ (Xtil(1:2)).*tanh( gain2*Xtil(1:2).^2 );

Ur = K \ (Xd + F1 + F2 );