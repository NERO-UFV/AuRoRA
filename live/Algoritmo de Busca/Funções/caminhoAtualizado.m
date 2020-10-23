function Caminho = caminhoAtualizado(CaminhoReal,Atalho)
%caminhoAtualizado Coloca o atalho no caminho real
cc_temp = find(Atalho(end) == CaminhoReal);
Caminho = [CaminhoReal(1:cc_temp - 1) Atalho CaminhoReal(cc_temp + 1:end)];
end

