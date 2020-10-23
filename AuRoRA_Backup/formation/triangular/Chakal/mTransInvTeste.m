function mTransInvTeste(obj, FiguraDesejada)
%% Carregar Parâmetros

xf = obj.pPos.Xd(1);
yf = obj.pPos.Xd(2);
psi = obj.pPos.Xd(3);
pf = obj.pPos.Xd(4);
qf = obj.pPos.Xd(5);
beta = obj.pPos.Xd(6);

%% Parâmetros auxiliares

rf = sqrt(pf^2 + qf^2 - 2*pf*qf*cos(beta));
h = sqrt(0.5*(pf^2 + qf^2 - 0.5*rf^2));
alpha = acos((pf^2 + h^2 - 0.25*rf^2)/(2*pf*h));

if obj.pSeq == 1 % Sequencia direta
    
    x1 = 2*h/3*cos(psi) + xf;
    y1 = 2*h/3*sin(psi) + yf;
    x2 = 2*h/3*cos(psi) - pf*cos(psi-alpha) + xf;
    y2 = 2*h/3*sin(psi) - pf*sin(psi-alpha) + yf;
    x3 = 2*h/3*cos(psi) - qf*cos(psi+beta-alpha) + xf;
    y3 = 2*h/3*sin(psi) - qf*sin(psi+beta-alpha) + yf;
    
else   % Sequencia inversa
    
    x1 = 2*h/3*cos(psi) + xf;
    y1 = 2*h/3*sin(psi) + yf;
    x2 = 2*h/3*cos(psi) - pf*cos(psi+alpha) + xf;
    y2 = 2*h/3*sin(psi) - pf*sin(psi+alpha) + yf;
    x3 = 2*h/3*cos(psi) - qf*cos(psi-beta+alpha) + xf;
    y3 = 2*h/3*sin(psi) - qf*sin(psi-beta+alpha) + yf;
    
end
disp('estouaqui')

FiguraDesejada(1:2,obj.pID) = [x1 y1]';
FiguraDesejada(1:2,obj.pID+1) = [x2 y2]';
FiguraDesejada(1:2,obj.pID+2) = [x3 y3]';

disp('Xd Calculado')
disp(size(FiguraDesejada))
disp(FiguraDesejada)

end