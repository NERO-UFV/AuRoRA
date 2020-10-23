%%% ALGORITMO Incrementa_zona_celula 
%%% FUNCAO RECURSIVA PARA INCREMENTAR AS CELULAS DO GRID, K VEZES
%%% AS CELULAS OCUPADAS DO GRID ORIRINAL POSSUEM FLAG = 1 (NAO PROCESSADAS)
%%% E FLAG = 3 (PROCESSADAS)
%%% AS CELULAS CONFIGURADAS COMO OBSTACULO RECEBEM VALOR 2
%%% A FUNCAO MOD E UTILIZADA PARA QUE O ALGORITMO SELECIONE TANTO AS
%%% CELULAS CONFIGURADAS FLAG = 2 QUANTO CELULAS LIVRE NAO PROCESSADAS
%%% FLAG = 0. AS CELULAS OBSTACULOS FLAG=3 SAO PROCESSADAS DE FORMA ITERATIVA NO
%%% ALGORITMO AplicaZonaSeguranca
%%% PARAMETROS 
%%%     grid : matriz com valores (0 = espaço livre, 1 = obstáculo 2 = zona livre que virou obstaculo
%%%            3 = obstaculo processado) 
%%%     k  : NUMERO DE VEZES QUE CADA CELULA OCUPADA SERÀ AUMENTADA  

function g = IncrementaZonaCelula( grid, i, j, max_i, max_j, k )
    if i>1
        if mod(grid(i-1,j),2) == 0
            grid(i-1,j) = 2;
            if k > 1
                grid = IncrementaZonaCelula(grid,i-1,j,max_i,max_j,k-1);
            end
        end
        
        if j>1 && mod(grid(i-1,j-1),2) == 0
            grid(i-1,j-1) = 2;
            if k>1
                grid = IncrementaZonaCelula(grid,i-1,j-1,max_i,max_j,k-1);
            end
        end
        if j<max_j && mod(grid(i-1,j+1),2) == 0
            grid(i-1,j+1) = 2;
            if k>1
                grid = IncrementaZonaCelula(grid,i-1,j+1,max_i,max_j,k-1);
            end
        end
    end    
    if i<max_i && mod(grid(i+1,j),2) == 0
        grid(i+1,j) = 2;
        if k>1
            grid = IncrementaZonaCelula(grid,i+1,j,max_i,max_j,k-1);
        end
        if j>1 && mod(grid(i+1,j-1),2) == 0
            grid(i+1,j-1) = 2;
            if k>1
                grid = IncrementaZonaCelula(grid,i+1,j-1,max_i,max_j,k-1);
            end
        end
        if j<max_j && mod(grid(i+1,j+1),2) == 0
            grid(i+1,j+1) = 2;
            if k>1
                grid = IncrementaZonaCelula(grid,i+1,j+1,max_i,max_j,k-1);
            end
        end
    end
    if j<max_j && mod(grid(i,j+1),2) == 0
        grid(i,j+1) = 2;
        if k>1
            grid = IncrementaZonaCelula(grid,i,j+1,max_i,max_j,k-1);
        end
    end
    if j>1 && mod(grid(i,j-1),2) == 0
        grid(i,j-1) = 2;
        if k>1
            grid = IncrementaZonaCelula(grid,i,j-1,max_i,max_j,k-1);
        end
    end
    g = grid;
end

