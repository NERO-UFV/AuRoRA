% Circunferencia
% AnguloZ = linspace(0,2*pi,200);
% X = sin(AnguloZ);
% Y = sin(2*AnguloZ);
% Z = 1 + cos(AnguloZ)/4;

% Linear
Ponto{1} = [0 0 0];
Ponto{2} = [3 3 3];
Parametro = linspace(0,1,200);
X = Ponto{2}(1).*ones(1,size(Parametro,2)) + (Ponto{1}(1) - Ponto{2}(1)).*Parametro;
Y = Ponto{2}(2).*ones(1,size(Parametro,2)) + (Ponto{1}(2) - Ponto{2}(2)).*Parametro;
Z = Ponto{2}(3).*ones(1,size(Parametro,2)) + (Ponto{1}(3) - Ponto{2}(3)).*Parametro;

hold on
plot3(X,Y,Z,'b.')
grid on
axis equal