function [Sucessor,Pos] = mapaSucessor(No,Mapa,Vertices)
%mapaSucessor entrega os nós resultantes da expansão do nó desejado
[eixoX,eixoY] = find((No(1) == Mapa) == 1);
cont = 1;
for i = -1:2:1
    try
        Sucessor(cont,:) = [Vertices(Mapa(eixoX,eixoY + i),:) No(1)];
        Pos{cont,1} = i;
        Pos{cont,2} = 'H';
        cont = cont + 1;
    end
end
for j = -1:2:1
    try
        Sucessor(cont,:) = [Vertices(Mapa(eixoX + j,eixoY),:) No(1)];
        Pos{cont,1} = j;
        Pos{cont,2} = 'V';
        cont = cont + 1;
    end
end
end

