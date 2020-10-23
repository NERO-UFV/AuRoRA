function [ind_desvio_inicio,flag_ameaca] = verifica_proximidade(C,obs_ativo,dist_min_obs,ind)

flag_ameaca = 1;
i = 1;
dist_desvio_inicio = 1000;
while dist_desvio_inicio > dist_min_obs
    if ind+i < size(C,2)
        dist_desvio_inicio = norm(C(1:2,ind+i)-obs_ativo(1:2));
    else
        flag_ameaca = 0;
        break
    end
    i = i + 1;
end
ind_desvio_inicio = ind+i;