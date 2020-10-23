function [Caminho,FECHADO] = LPAstar_2(Sinit,Sg,Mapa,Vertices,FECHADO)
%A* calcula o menor caminho de Sinit ate Sg, dado um Mapa
% Sinit = Vertices(4,:);
% Sg = Vertices(22,:);
% Mapa = Nomes;
Custos = evalin('base','Mapa.Plot.Custo');
ABERTO = [];
if nargin == 4
    FECHADO = [];
    ABERTO(1,:) = [Sinit 0 0 distanciaManhattan(Sinit,Sg) distanciaManhattan(Sinit,Sg)];
    FECHADO(1,:) = [Sinit 0 0 distanciaManhattan(Sinit,Sg) distanciaManhattan(Sinit,Sg)];
    Custos(FECHADO(end,1)).String = num2str(FECHADO(end,5));
else
    ABERTO = FECHADO(find(Sinit(1) == FECHADO(:,1)):end,:);
    ABERTO = sortrows(ABERTO,6);
end

while size(ABERTO,1) ~= 0
    n = ABERTO(1,:);
    ABERTO = ABERTO(2:end,:);
    if n(1) == Sg(1)
        posAnterior = find(n(1)==FECHADO(:,1));
        Caminho = zeros(1,n(5) + 1);
        Caminho(end) = [n(1)];
        for i = 1:n(1,end)
            Caminho(end - i) = [FECHADO(posAnterior,4)];
            posAnterior = find(FECHADO(posAnterior,4)==FECHADO(:,1));
        end
%         disp(FECHADO)
        break
    end
    Sucessor = mapaSucessor(n,Mapa,Vertices);
    for i = 1:size(Sucessor,1)
        c = Sucessor(i,:);
        if sum(c(:,1) == FECHADO(:,1)) == 0
            f_temp = (n(1,end) - distanciaManhattan(n,Sg)) + 1 + distanciaManhattan(c,Sg);
            ABERTO(end + 1,:) = [c (n(5) + 1) distanciaManhattan(c,Sg) f_temp];
            f_temp = (n(1,end) - distanciaManhattan(n,Sg)) + 1 + distanciaManhattan(c,Sg);
            FECHADO(end + 1,:) = [c (n(5) + 1) distanciaManhattan(c,Sg) f_temp];
        end
        pos = find(c(:,1) == FECHADO(:,1));
        f_temp = (n(1,end) - distanciaManhattan(n,Sg)) + 1 + distanciaManhattan(c,Sg);
        if sum(c(:,1) == FECHADO(:,1)) ~= 0 && FECHADO(pos,end) > f_temp
            FECHADO(pos,end) = f_temp;
        end
        Custos(FECHADO(end,1)).String = num2str(FECHADO(end,5));
    end
    ABERTO = sortrows(ABERTO,6);
end
end
