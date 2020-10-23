%%% ALGORITMO AplicaZonaSeguranca 
%%% AUMENTA AS AREAS EM TORNO DE CELULAS DO GRID QUE ESTAO OCUPADAS. PARA CADA CELULA
%%% COM VALOR = 1 , AS 8 CELULAS ADJACENTES SERÂO DEFINIDAS COMO OCUPADAS
%%% PARAMETROS 
%%%     grid : matriz com valores biarios (0 = espaço livre, 1 = obstáculo) 
%%%     k  : NUMERO DE VEZES QUE CADA CELULA OCUPADA SERÀ AUMENTADA  

function g = AplicaZonaSeguranca( grid , k )
    t = size(grid);
    for i=1:t(1)
        for j=1:t(2)
            if grid(i,j)==1                                
                grid(i,j)=3;
                grid = IncrementaZonaCelula(grid,i,j,t(1),t(2),k);
            end
        end
    end
    g = floor(grid/2);
end