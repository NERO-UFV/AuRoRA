function mConexoesCriar(obj)

% Função que cria as conexões com os demais robôs que compõem a formação
% baseada em consenso
%
% Esta função somente será chamada, quando um experimento estiver sendo
% realizado. 

obj.pVC.ConexoesFlag = 1;

for ii = 1:obj.pVF.nID    
    obj.pVC.Conexoes(ii) = udp('255.255.255.255', 'localPort', 25000+ii);    
end

fopen(obj.pVC.Conexoes);

end