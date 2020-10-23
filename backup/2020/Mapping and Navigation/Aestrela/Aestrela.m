%%% ALGORITMO A*
%%% ENCONTRA O MENOR CAMINHO ENTRE DUAS CELULAS DO GRID
%%% PARAMETROS
%%%     occ_grid : objeto Occupancy Grid fornecido pelo matlab
%%%     grid : matriz com valores biarios (0 = espaco livre, 1 = obstaculo)
%%%     pos_inicio  : vetor com indices x e y da celula de inicio
%%%     pos_destino : vetor com indices x e y da celula de destino
%%% RETORNO
%%%     caminho: matriz [tx2] com as posições do grid do caminho ótimo.
%%% IMPORTANTE
%%% 
%%% Os nós são identificados na Lista Aberta e Fechada pelo número
%%% referente a sequencia dentro da matriz, seguindo a ordem por
%%% linha (Ex: nó (2,2) de uma matriz 4x4 possui indice 6
%%%
function [caminho] = Aestrela( occ_grid, grid, pos_inicio, pos_destino )
    ListaAberta = [];
    ListaFechada = [];
    inicioX = pos_inicio(1);
    inicioY = pos_inicio(2);
    fimX = pos_destino(1);
    fimY = pos_destino(2);
    tamX = size(grid,1);
    tamY = size(grid,2);
    tamT = tamX*tamY;
    ListaFechada = zeros(tamT,8);
    ListaAberta = ListaFechada;
    %adiciona as celulas que sao obstaculo na lista fechada
    for i=1:tamX
        for j=1:tamY
            if grid(i,j) == 1
                posF = (i-1)*tamY+j;
                ListaFechada(posF,:) = [1 i j i j 0 0 0];
            end
        end
    end
    %Marca o nó inicial como Aberto
    ListaAberta((inicioX-1)*tamY+inicioY,:) = [1 inicioX inicioY inicioX inicioY 0 0 0];
    nodoX = inicioX;
    nodoY = inicioY;
    listaNaoVazia = true;
    while ((nodoX ~= fimX) || (nodoY ~= fimY)) && (listaNaoVazia == true)
        posMin = -1;
        minFN = inf;
        %Pega o nó marcado como aberto com menor valor de f(n)
        for i=1:size(ListaAberta,1)
            if ListaAberta(i,1) == 1 && ListaAberta(i,8) < minFN
                posMin = i;
                minFN = ListaAberta(i,8);
            end
        end
        %Nenhum nó marcado como aberto, termina
        if posMin == -1
            listaNaoVazia = false;
        else
            nodoX = ceil(posMin/tamY);
            if posMin <= tamY
                nodoY = posMin;
            else
                nodoY = rem(posMin,tamY);
                if nodoY == 0
                    nodoY = tamY;
                end
            end
            if nodoX == fimX && nodoY == fimY
                listaNaoVazia = false;
            else
                %marca o nó como fechado, expande os sucessores (adjacentes
                %no grid)
                ListaAberta(posMin,1) = false;
                ListaFechada(posMin,:) = ListaAberta(posMin,:);
                ListaFechada(posMin,1) = true;
                [ListaAberta,ListaFechada] = ExpandeNodo(occ_grid,ListaAberta,ListaFechada,posMin,tamX,tamY,fimX,fimY);
            end
        end
    end
    caminho = [];
    if posMin > 0
        ListaFechada(posMin,:) = ListaAberta(posMin,:);
        while nodoX ~= inicioX || nodoY ~= inicioY
            caminho = [caminho; nodoX nodoY];
            antecessor = (nodoX-1)*tamY+nodoY;
            nodoX = ListaFechada(antecessor,4);
            nodoY = ListaFechada(antecessor,5);
        end
        antecessor = (nodoX-1)*tamY+nodoY;
        nodoX = ListaFechada(antecessor,4);
        nodoY = ListaFechada(antecessor,5);
        caminho = [caminho; nodoX nodoY];
    end
end

