function [xq,yq,rota,tem_caminho,grid,curva,poly] = ProcuraCaminho( ij_origem, ij_destino, occ_grid )
%PROCURACAMINHO Summary of this function goes here
%   Detailed explanation goes here      
    curva = [];
    poly = [];
    %grid = AplicaZonaSeguranca(double(occ_grid.mod),ceil(0.4 / (1/occ_grid.og.Resolution)));    
    %k = ceil(0.4 / (1/occ_grid.og.Resolution));
     k=4;
     grid = AplicaZonaSeguranca(double(occ_grid.mod),k);
%     k=4;
    for i=ij_origem(1)-k:ij_origem(1)+k
        for j=ij_origem(2)-k:ij_origem(2)+k
            grid(i,j) = 0;
        end
    end
    
    % PARA TESTAR %
    for i=ij_destino(1)-k:ij_destino(1)+k
        for j=ij_destino(2)-k:ij_destino(2)+k
            grid(i,j) = 0;
        end
    end
    %grid = 0;
    
    % FIM TESTE %
    
    grid(ij_origem(1),ij_origem(2)) = 0;
    grid(ij_destino(1),ij_destino(2)) = 0;
    rota = Aestrela(occ_grid.og,grid,ij_origem,ij_destino);
    rota = flipud(rota);    
    if size(rota,1) == 0
        xq = [];        
        yq = [];
        tem_caminho = false;
    else
        caminho = [];
        for i = 1:size(rota,1)
            caminho = [caminho; grid2world(occ_grid.og,[rota(i,1) rota(i,2)])];
            %disp(grid2world(og,[linha coluna]));
        end
        %caminho = pp_curva2(caminho);
        for i=1:size(caminho,1)
            ij = world2grid(occ_grid.og,[caminho(i,1) caminho(i,2)]);
            grid(ij(1),ij(2)) = 0.5;
        end
        curva = [];
        %[xq,yq] = ConstroiCurva(caminho);
        for i=1:size(caminho,1)-1
            if caminho(i,1) == caminho(i+1,1)
                count = 1;
                aux = i+1;
                v = caminho(i,1);
                while aux < size(caminho,1) && caminho(aux,1) ==  v
                    count = count + 1;
                    aux = aux + 1;
                end
                if aux > size(caminho,1)
                    if caminho(i-1,1) > caminho(i,1)
                        caminho(i:aux-1,1) = linspace(caminho(i-1,1)-0.1,caminho(1,1)-0.1,count);
                    else
                        caminho(i:aux-1,1) = linspace(caminho(i-1,1)+0.1,caminho(1,1)+0.1,count);
                    end
                else
                    if caminho(aux,1) > caminho(i-1,1)
                        caminho(i:aux-1,1) = linspace(caminho(i,1)-0.1,caminho(aux,1)+0.1,count);
                    else
                        caminho(i:aux-1,1) = linspace(caminho(i,1)+0.1,caminho(aux,1)-0.1,count);
                    end
                end
            end
        end
        [xq,yq] = ConstroiCurva(caminho);
        tem_caminho=true;
    end   
end

