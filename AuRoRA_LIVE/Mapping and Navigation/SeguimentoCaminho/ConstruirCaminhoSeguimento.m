function Rota = ConstruirCaminhoSeguimento(infoRota)
Rota = zeros(12,length(infoRota));
Rota(1:2,:) = infoRota';
Rota(7:8,:) = [diff(infoRota)' [0;0]]*10;
Rota(6,:)   = atan2(Rota(8,:),Rota(7,:));
end