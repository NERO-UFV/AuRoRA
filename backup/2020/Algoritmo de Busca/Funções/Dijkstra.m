function Caminho = Dijkstra(Sinit,Sg,Mapa,Vertices)
%Dijkstra calcula o menor caminho de Sinit ate Sg, dado um Mapa
% Sinit = Vertices(4,:);
% Sg = Vertices(22,:);
% Mapa = Nomes;

ABERTO = [];
FECHADO = [];
ABERTO(1,:) = [Sinit 0 0];
FECHADO(1,:) = [Sinit 0 0];
while size(ABERTO,1) ~= 0
    n = ABERTO(1,:);
    ABERTO = ABERTO(2:end,:);
    if n(1) == Sg(1)
        posAnterior = find(n(1)==FECHADO(:,1));
        Caminho = zeros(1,n(1,end) + 1);
        Caminho(end) = [n(1)];
        for i = 1:n(1,end)
            Caminho(end - i) = [FECHADO(posAnterior,4)];
            posAnterior = find(FECHADO(posAnterior,4)==FECHADO(:,1));
        end
        disp(FECHADO)
        break
    end
    Sucessor = mapaSucessor(n,Mapa,Vertices);
    for i = 1:size(Sucessor,1)
        c = Sucessor(i,:);
        if sum(c(:,1) == FECHADO(:,1)) == 0
            ABERTO(end + 1,:) = [c n(1,end) + 1];
            FECHADO(end + 1,:) = [c n(1,end) + 1];
        end
        pos = find(c(:,1) == FECHADO(:,1));
        if sum(c(:,1) == FECHADO(:,1)) ~= 0 && FECHADO(pos,end) > n(1,end) + 1
            FECHADO(pos,end) = n(1,end) + 1;
        end
        ABERTO = sortrows(ABERTO,5);
    end
end
end

