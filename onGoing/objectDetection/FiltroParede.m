function [d2] = FiltroParede(d1,theta1,theta2)
%% Função para filtrar valores das leituras
%Essa função vai ter de entrada a leitura do sensor inferior, onde d1 é a distância lida pelo
%sensor
%o angulo entre eles ( theta1) e o angulo
%máximo de inclinação que gostaria que se aceitasse a leitura
%                theta2/
%                |    /
%                |   / 
%                |  /
% theta1         | /
% _______________|/
%A função irá calcular a razão entre a leitura do sensor superior e do sensor inferior no caso de
%ter uma parede de theta2 graus de inclinação e retorna essa razão
%Pela lei dos senos temos que d1/sin(180 -(90+theta2)-theta1)=d2/sin(90+theta2)
d2=d1*sind(90+theta2)/sind(180-(90+theta2)-theta1);


end

