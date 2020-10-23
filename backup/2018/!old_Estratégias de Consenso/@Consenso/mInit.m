function mInit(obj)
% Variáveis de Formação
obj.pVF.X    = zeros(12,1); % Postura dos robôs
obj.pVF.SC   = [];          % Sinal de controle Global
obj.pVF.Ei   = zeros(3,1);  % Estado de informação corrente
obj.pVF.Eid  = zeros(3,1);  % Estado de informação desejado
obj.pVF.dEi  = zeros(3,1);  % Derivada do estado de informação corrente
obj.pVF.dEid = zeros(3,1);  % Valor de referência para a equação de consenso
obj.pVF.Madj = [];          % Matriz de adjacência
obj.pVF.ID   = 0;           % Identificação do robô
obj.pVF.nID  = 0;           % Número de robôs

obj.pVF.Xd   = zeros(3,1);  % Valores de referência a serem adotados
obj.pVF.dXd  = zeros(3,1);  % como posição e velocidade do robô

obj.pVF.XF   = zeros(3,1);  % Formação desejada ou ponto de encontro

%Variáveis de Comunicação
% obj.pVC.Conexoes = 0;      % Comunicação com a rede UDP para buscar
                            % dados com os demais robôs (Abertura portas de
                            % comunicação)
                            
obj.pVC.ConexoesFlag = 0;  % Flag de comunicação: 0 - Simulação, 1 - Experimento                            
                            

                       
end