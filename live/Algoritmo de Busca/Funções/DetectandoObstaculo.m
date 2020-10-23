function DetectandoObstaculo(ObsID)
%DetectandoObstaculo detecta o obstaculo e retorna o nó que virou obstaculo
%   Detailed explanation goes here
Dist = evalin('base','Dist');
Tamanho = evalin('base','Tamanho');
Deslocamento = evalin('base','Deslocamento');
A = evalin('base','A');
if A{ObsID}.pPos.X(3) < 0.4
    pt = A{ObsID}.pPos.X(1:2) + Deslocamento;
    pt = floor(pt/Dist);
    assignin('base','obsPoint',pt')
else
    pt = [];
    assignin('base','obsPoint',pt)
end
end

