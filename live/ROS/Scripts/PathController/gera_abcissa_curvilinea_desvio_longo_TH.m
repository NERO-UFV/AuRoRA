function C_desvio = gera_abcissa_curvilinea_desvio_longo_TH(pontoInicial,pontoFinal,obs_ativo,inc_desvio,dist_min_obs)

% Acha o ângulo entre o arco entre o obstáculo e os pontos
% iniciais e finais do novo caminho
ang_obs_inicio = atan2(obs_ativo(2) - pontoInicial(2,1),obs_ativo(1) - pontoInicial(1,1));
% ang_obs_final = atan2(pontoFinal(2,1) - obs_ativo(2),pontoFinal(1,1) - obs_ativo(1))*180/pi;

TH = [cos(ang_obs_inicio) -sin(ang_obs_inicio) 0 obs_ativo(1);
    sin(ang_obs_inicio) cos(ang_obs_inicio) 0 obs_ativo(2);
        0           0                       1 obs_ativo(3);
        0           0       0    1    ];
    
pontoInicial_zero = TH\[pontoInicial;1];
pontoFinal_zero = TH\[pontoFinal;1];
obs_ativo_zero = TH\[obs_ativo;1];

% Define a abcissa curvilínea do desvio
if pontoFinal_zero(2) >= 0
    inc = inc_desvio;
    ang_inic = -180;
else
    inc = -inc_desvio;
    ang_inic = 180;
end
ang_final = atan2(pontoFinal_zero(2),pontoFinal_zero(1))*180/pi;
s_desvio = [ang_inic:inc:ang_final];

% Define parâmetros do novo caminho de desvio
RaioX_desvio = dist_min_obs;
RaioY_desvio = RaioX_desvio;
CentroX_desvio = obs_ativo(1);
CentroY_desvio = obs_ativo(2);

% Define as equações da curva de desvio
x_desvio = RaioX_desvio*cos(pi*s_desvio/180) + obs_ativo_zero(1);
y_desvio = RaioY_desvio*sin(pi*s_desvio/180) + obs_ativo_zero(2);
z_desvio = 0*ones(1,length(s_desvio));

C_desvio_zero = [x_desvio; y_desvio; z_desvio];

C_desvio_aux = TH*[C_desvio_zero;ones(1,length(s_desvio))];
C_desvio = C_desvio_aux(1:3,:);
