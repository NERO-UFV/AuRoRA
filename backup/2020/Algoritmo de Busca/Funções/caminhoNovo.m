function Caminho = caminhoNovo(Caminho,temp_cc)
%caminhoNovo atualiza o caminho encontrado para nao dar erro
% temp_cc é o nome do nó
% novo_cc é o lugar do nó no vetor caminho

Mapa = evalin('base','Nomes');
Vertices = evalin('base','Vertices');

if size(find(temp_cc == Caminho),2) ~= 0
    A = 0;
else
    Sucessor = mapaSucessor(temp_cc,Mapa,Vertices);
    for i = 1:size(Sucessor,1)
        novo_cc = find(Sucessor(i,1) == Caminho);
        if size(novo_cc,2) ~= 0
            C_temp = Caminho;
            C_temp(novo_cc + 1) = temp_cc;
            C_temp((novo_cc + 2):(end + 2)) = Caminho(novo_cc:end);
            break
        end
    end
    Caminho = C_temp;
end


end

