function [C_desvio,ind_caminho_obs_final] = gera_caminho_desvio_mudanca(X,C_normal,obs_ativo,dist_min_obs,inc_desvio,ind,obs_ativo_antigo)

% Esta função descobre o índice do caminho normal mais perto do obstáculo.
% Tal índice será usado para começar o laço para se descobrir o índice do
% final do desvio
[dist_caminho_obs, ind_caminho_obs] = calcula_ponto_proximo(C_normal(1:2,:),obs_ativo(1:2));

% Este laço utiliza o índice mais próximo do obstáculo, para
% descobrir o primeiro índice depois desse onde a distância ao
% obstáculo é maior que o limiar de desvio
i = 1;
dist_caminho_obs_final = 0;
while dist_caminho_obs_final < dist_min_obs
    dist_caminho_obs_final = norm(C_normal(1:2,ind_caminho_obs+i)-obs_ativo(1:2));
    i = i + 1;
end
ind_caminho_obs_final = ind_caminho_obs+i;

% Define parâmetros do novo caminho de desvio
RaioX_desvio = dist_min_obs;
RaioY_desvio = RaioX_desvio;
CentroX_desvio = obs_ativo(1);
CentroY_desvio = obs_ativo(2);

s_desvio_curto = gera_abcissa_curvilinea_desvio_curto(X(1:2,1),C_normal(1:2,ind_caminho_obs_final),obs_ativo,inc_desvio);

% Define as equações da curva de desvio curta
x_desvio_curto = RaioX_desvio*cos(pi*s_desvio_curto/180) + CentroX_desvio;
y_desvio_curto = RaioY_desvio*sin(pi*s_desvio_curto/180) + CentroY_desvio;
z_desvio_curto = 0*ones(1,length(s_desvio_curto));
C_desvio_curto = [x_desvio_curto; y_desvio_curto; z_desvio_curto];

s_desvio_longo = gera_abcissa_curvilinea_desvio_longo(X(1:2,1),C_normal(1:2,ind_caminho_obs_final),obs_ativo,inc_desvio);

% Define as equações da curva de desvio longa
x_desvio_longo = RaioX_desvio*cos(pi*s_desvio_longo/180) + CentroX_desvio;
y_desvio_longo = RaioY_desvio*sin(pi*s_desvio_longo/180) + CentroY_desvio;
z_desvio_longo = 0*ones(1,length(s_desvio_longo));
C_desvio_longo = [x_desvio_longo; y_desvio_longo; z_desvio_longo];

min_dist_curto = 1000;
for i = 1:size(C_desvio_curto,2)
    min_dist_curto_temp = norm(C_desvio_curto(1:2,i)-obs_ativo_antigo(1:2));
    if min_dist_curto_temp < min_dist_curto
        min_dist_curto = min_dist_curto_temp;
    end
end

min_dist_longo = 1000;
for i = 1:size(C_desvio_longo,2)
    min_dist_longo_temp = norm(C_desvio_longo(1:2,i)-obs_ativo_antigo(1:2));
    if min_dist_longo_temp < min_dist_longo
        min_dist_longo = min_dist_longo_temp;
    end
end

if min_dist_curto > min_dist_longo || min_dist_curto > dist_min_obs
    C_desvio = C_desvio_curto;
else
    C_desvio = C_desvio_longo;
end