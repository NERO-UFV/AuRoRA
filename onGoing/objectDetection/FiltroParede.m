function [d2] = FiltroParede(d1,theta1,theta2)
%% Fun��o para filtrar valores das leituras
%Essa fun��o vai ter de entrada a leitura do sensor inferior, onde d1 � a dist�ncia lida pelo
%sensor
%o angulo entre eles ( theta1) e o angulo
%m�ximo de inclina��o que gostaria que se aceitasse a leitura
%                theta2/
%                |    /
%                |   / 
%                |  /
% theta1         | /
% _______________|/
%A fun��o ir� calcular a raz�o entre a leitura do sensor superior e do sensor inferior no caso de
%ter uma parede de theta2 graus de inclina��o e retorna essa raz�o
%Pela lei dos senos temos que d1/sin(180 -(90+theta2)-theta1)=d2/sin(90+theta2)
d2=d1*sind(90+theta2)/sind(180-(90+theta2)-theta1);


end

