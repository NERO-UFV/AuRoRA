function Xd = SeguirCaminho(Rota,X)
%  Ganho de controladores
Vmax = 0.20;
% Constantes do Controlador de Caminhos
Khr1 = [0.2 0 0 0; 0 0.2 0 0; 0 0 0.2 0; 0 0 0 0.2];
Khr2 = [.4 0 0 0; 0 .4 0 0; 0 0 0.4 0; 0 0 0 0.4];
Xr = zeros(12,1);
Xd = zeros(12,1);

% Determina��o do ponto mais pr�ximo entre o Rota e o rob�
% Criar sistema de janelamento [kk_min kk kk_max]
rho_min = 100000;
for kk = 15:length(Rota)
    rho = norm(X(1:3)-Rota(1:3,kk));
    if rho < rho_min
        rho_min = rho;
        pos = kk;
    end
end
Xc = Rota(:,pos);

% C�lculo do ponto mais pr�ximo ao Rota em coordenadas polares
rho   = rho_min;
alpha = atan2(Xc(9),norm(Xc(7:8)));
beta  = atan2(Xc(8),Xc(7));

% C�lculo da velocidade do rob� sobre o Rota
% Velocidade Desejada muda de acordo com o erro de dist�ncia
V = Vmax/(1+2*rho);

% Erro nas refer�ncias de seguimento de Rotas
Xf   = Rota(:,end); % �ltimo ponto
if norm(Xf(1:2)-Xc(1:2)) < 0.050
    Xc = Xf;
    V = 0;
end

Xtil = Xc - X;

Xr([7 8 9 12]) = [V*cos(alpha)*cos(beta); V*cos(alpha)*sin(beta); V*sin(alpha); Xtil(12)];

KM = [cos(X(6)) -sin(X(6)) 0 0; sin(X(6)) cos(X(6)) 0 0; 0 0 1 0; 0 0 0 1];

uSC = KM\(Xr([7 8 9 12],1) + Khr1*tanh(Khr2*Xtil([1 2 3 6])));

% C�lculo do sinal de controle para alterar a velocidade desejada
% Atribuindo posi��o desejada
Xd([1 2 3 6])  = Xc([1 2 3 6]);
Xd([7 8 9 12]) = KM*uSC;
end