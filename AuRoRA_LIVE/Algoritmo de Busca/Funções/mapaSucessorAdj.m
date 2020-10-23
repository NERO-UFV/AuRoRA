function Sucessor = mapaSucessorAdj(No,Adjacencia,Vertices)
%mapaSucessor entrega os nós resultantes da expansão do nó desejado
Sucessor = find(Adjacencia(No(1),:)==1);
% Sucessor = Sucessor(Sucessor>No(1));
Sucessor = Vertices(Sucessor,:);
end

