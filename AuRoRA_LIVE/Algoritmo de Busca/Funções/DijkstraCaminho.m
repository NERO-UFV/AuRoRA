function Atalho = DijkstraCaminho(Sinit,Caminho,Mapa,Vertices)
%DijkstraCaminho calcula o menor atalho de Sinit ate o Caminho, dado um Mapa
% temp_cc é o nome do nó
% novo_cc é o lugar do nó no vetor caminho
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
    if size(find(n(1) == Caminho),2) ~= 0
%     if n(1) == Sg(1)
        posAnterior = find(n(1)==FECHADO(:,1));
        Atalho = zeros(1,n(1,end) + 1);
        Atalho(end) = n(1);
        for i = 1:n(1,end)
            Atalho(end - i) = FECHADO(posAnterior,4);
            posAnterior = find(FECHADO(posAnterior,4)==FECHADO(:,1));
        end
%         disp(FECHADO)
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

