function [dist,ind] = calcula_ponto_proximo(curva,ponto)

dist_min = 1000;
for i = 1:length(curva)
    dist_atual = norm(curva(:,i)-ponto);
    if dist_atual < dist_min
        dist_min = dist_atual;
        dist_min_ind = i;
    end
end

dist = dist_min;
ind = dist_min_ind;