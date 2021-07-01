function Drone = cPathFollowing(Drone,Path)

X = Drone.pPos.X;

%  Ganho de controladores
Vmax = 0.50;

% Constantes do Controlador de Caminhos
Khr1 = [0.2 0 0 0; 0 0.2  0 0; 0 0 0.20 0; 0 0 0 0.50];
Khr2 = [.75 0 0 0; 0 0.75 0 0; 0 0 0.75 0; 0 0 0 0.75];
Xr = zeros(12,1);
Xd = zeros(12,1);

% Determinação do ponto mais próximo entre o Rota e o robô
% Criar sistema de janelamento [kk_min kk kk_max]
rho_min = 100000;
for kk = 2:length(Path)
    rho = norm(X(1:3)-Path(1:3,kk));
    if rho < rho_min
        rho_min = rho;
        pos = kk;
    end
end
Xc = Path(:,pos);

% Cálculo do ponto mais próximo ao Rota em coordenadas polares
rho   = rho_min;
alpha = atan2(Xc(9),norm(Xc(7:8)));
beta  = atan2(Xc(8),Xc(7));

% Cálculo da velocidade do robô sobre o Rota
% Velocidade Desejada muda de acordo com o erro de distância
V = Vmax/(1+2*rho);

% % Erro nas referências de seguimento de Rotas
Xf   = Path(:,end); % último ponto
if norm(Xf(1:2)-Xc(1:2)) < 0.050
    Xc = Xf;
    V = 0;
end

Xtil = Xc - X;

Xr([7 8 9 12]) = [V*cos(alpha)*cos(beta); V*cos(alpha)*sin(beta); V*sin(alpha); Xtil(12)];

% Modelo Cinemático
KM = [cos(X(6)) -sin(X(6)) 0 0; sin(X(6)) cos(X(6)) 0 0; 0 0 1 0; 0 0 0 1];
uSC = KM\(Xr([7 8 9 12],1) + Khr1*tanh(Khr2*Xtil([1 2 3 6])));

% Cálculo do sinal de controle para alterar a velocidade desejada
% Atribuindo posição desejada
Xd([1 2 3 6])  = Xc([1 2 3 6]);
Xd([7 8 9 12]) = KM*uSC;

Drone.pPos.Xd = Xd;



