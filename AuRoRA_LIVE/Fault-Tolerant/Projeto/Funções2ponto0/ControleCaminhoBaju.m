function obj = ControleCaminhoBaju(caminho,cc,obj)
% ControleCaminhoBaju, Usando o controle de caminho do artigo 
% (DOI: 10.1109/ICCA.2011.6138018)

% Constantes do Controlador de Caminhos
Khr1 = [0.2 0 0; 0 0.2  0; 0 0 0.20];
Khr2 = [0.75 0 0; 0 0.75 0; 0 0 0.75];

% Definindo o valor de rho
caminho.RHO = sqrt((caminho.X(1,cc) - obj.pPos.X(1))^2 + (caminho.X(2,cc) - obj.pPos.X(2))^2 + (caminho.X(3,cc) - obj.pPos.X(3))^2);

% Definindo o valor de alfa
caminho.ALFA = atan2(caminho.dX(3,cc),sqrt(caminho.dX(1,cc)^2 + caminho.dX(2,cc)^2));
% Controle de caminho
caminho.BETA = atan2(caminho.dX(2,cc),caminho.dX(1,cc));

% Definindo o valor constante da velocidade
caminho.V = caminho.VMAX/(1 + caminho.K*caminho.RHO);

% Definindo as derivadas
caminho.dXr = [caminho.V*cos(caminho.ALFA)*cos(caminho.BETA);
               caminho.V*cos(caminho.ALFA)*sin(caminho.BETA);
               caminho.V*sin(caminho.ALFA);
               caminho.dX(4,cc)];
           
caminho.Xtil = caminho.X(1:3,cc) - obj.pPos.X(1:3);


obj.pPos.Xd(1:3) = caminho.X(1:3,cc);
obj.pPos.Xd(7:9) = caminho.dXr(1:3) + Khr1*tanh(Khr2*caminho.Xtil([1 2 3]));
end

