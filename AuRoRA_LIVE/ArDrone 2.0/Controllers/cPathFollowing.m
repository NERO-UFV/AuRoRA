function [Drone,Path,rho] = cPathFollowing(Drone,Path,rhod)
% 

% Constantes do Controlador de Caminhos
Khr1 = [0.2 0 0; 0 0.2  0; 0 0 0.20];
Khr2 = [0.75 0 0; 0 0.75 0; 0 0 0.75];
% Xd = zeros(12,1);

% Navegar para o primeiro ponto do caminho
% Verificar se o erro entre o caminho e a posição atual do robô é inferior
% a um limiar (0.05) rhod
if norm(Drone.pPos.X(1:3)-Path.X(1:3,Path.Pos)) < rhod
    Path.Pos = Path.Pos + 1;    
    if Path.Pos > Path.Size-1
        Path.Pos = 1;
    end
end

% Cálculo do ponto mais próximo ao Rota em coordenadas polares
rho   = norm(Path.X(1:3,Path.Pos)-Drone.pPos.X(1:3));
alpha = atan2(Path.dX(3,Path.Pos),norm(Path.dX(1:2,Path.Pos)));
beta  = atan2(Path.dX(2,Path.Pos),Path.dX(1,Path.Pos));

% Cálculo da velocidade do robô sobre o Rota
% Velocidade Desejada muda de acordo com o erro de distância
V = Path.Vmax/(1+2*rho);

Path.dXr = [V*cos(alpha)*cos(beta); V*cos(alpha)*sin(beta); V*sin(alpha)];

Xtil = Path.X(1:3,Path.Pos) - Drone.pPos.X(1:3);

% Modelo Cinemático
KM = [cos(Drone.pPos.X(6)) -sin(Drone.pPos.X(6)) 0; sin(Drone.pPos.X(6)) cos(Drone.pPos.X(6)) 0; 0 0 1];
uSC = KM\(Path.dXr + Khr1*tanh(Khr2*Xtil([1 2 3])));

Drone.pPos.Xd(1:3) = Path.X(1:3,Path.Pos);
Drone.pPos.Xd(6) = beta;
Drone.pPos.Xd(7:9) = uSC;
% Path.dXr + Khr1*tanh(Khr2*Xtil([1 2 3]));

% % Alterar a velocidade sobre o caminho em função da proximidade tangencial
% 
% 
% % % Cálculo do ponto mais próximo ao Rota em coordenadas polares
% % rho   = norm(Path.Xc(1:3)-Drone.pPos.X(1:3));
% % alpha = atan2(Path.Xc(9),norm(Path.Xc(7:8)));
% % beta  = atan2(Path.Xc(8),Path.Xc(7));
% %
% % % Cálculo da velocidade do robô sobre o Rota
% % % Velocidade Desejada muda de acordo com o erro de distância
% % V = Path.Vmax/(1+2*rho);
% %
% % Xtil = Path.Xc - Drone.pPos.X
% %
% % Path.Xr([7 8 9 12]) = [V*cos(alpha)*cos(beta); V*cos(alpha)*sin(beta); V*sin(alpha); Xtil(12)];
% %
% % % Modelo Cinemático
% % KM = [cos(Drone.pPos.X(6)) -sin(Drone.pPos.X(6)) 0 0; sin(Drone.pPos.X(6)) cos(Drone.pPos.X(6)) 0 0; 0 0 1 0; 0 0 0 1];
% % uSC = KM\(Path.Xr([7 8 9 12],1) + Khr1*tanh(Khr2*Xtil([1 2 3 6])));
% %
% % % Cálculo do sinal de controle para alterar a velocidade desejada
% % % Atribuindo posição desejada
% % Xd([1 2 3 6])  = Path.Xc([1 2 3 6]);
% % Xd([7 8 9 12]) = KM*uSC;
% 
% %Drone.pPos.Xd = Path.Xc;


