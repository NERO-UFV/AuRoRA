function clickFncfDD(src,~)
%clickFunctionfloofDistDeslocamento
Dist = evalin('base','Dist');
Tamanho = evalin('base','Tamanho');
Deslocamento = evalin('base','Deslocamento');
pt = get(gca,'CurrentPoint') + Deslocamento;
pt = floor(pt/Dist);
assignin('base','obsPoint',pt)
end
