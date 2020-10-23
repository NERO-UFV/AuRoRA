%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% PLOT INICIAL DO EXPERIMENTO
% Pegando os valores das variaveis
psi = evalin('base','psi');
A = evalin('base','A');
P = evalin('base','P');
raio = evalin('base','raio');

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Esfera limite
E = 0:pi/32:pi;
I = ones(1,2);

X_E = [raio(1)*cos(psi)*cos(E);
       raio(1)*sin(psi)*cos(E);
       raio(1)*sin(E)];
   
X_C = [raio(2)*cos(psi)*I;
       raio(2)*sin(psi)*I;
       [E(1) E(end)]];

% R rotaciona em relação ao eixo-z
R = [cos(pi/10)  -sin(pi/10)  0;
     sin(pi/10)  cos(pi/10)   0;
     0           0            1];

% Armazenando o valor anterior da esfera
X_Ea = X_E;
X_E = R*X_E;
% Armazenando o valor anterior do cilindro
X_Ca = X_C;
X_C = R*X_C;
 
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Figura da simulação
figure
hold on
A{1}.mCADplot;
A{2}.mCADplot;
P.mCADplot(1,'b');
grid on

for j=1:10
for i=1:(size(X_E,2)-1)
    patch([X_Ea(1,i) X_Ea(1,i+1) X_E(1,i+1) X_E(1,i)],...
        [X_Ea(2,i) X_Ea(2,i+1) X_E(2,i+1) X_E(2,i)],...
        [X_Ea(3,i) X_Ea(3,i+1) X_E(3,i+1) X_E(3,i)],'r',...
        'EdgeColor','k','FaceAlpha',0.5);
end
    X_Ea = X_E;
    X_E = R*X_E;
end

for j = 1:10*2
    patch([X_Ca(1,1) X_Ca(1,2) X_C(1,2) X_C(1,1)],...
        [X_Ca(2,1) X_Ca(2,2) X_C(2,2) X_C(2,1)],...
        [X_Ca(3,1) X_Ca(3,2) X_C(3,2) X_C(3,1)],'b',...
        'EdgeColor','k','FaceAlpha',0.5);
    X_Ca = X_C;
    X_C = R*X_C;
end

% Configurações da visulização do ambiente
AX = raio(1)*2;
axis([-AX AX -AX AX 0 AX])
axis equal
az = psi*180/pi;
el = 45;
view([az el])