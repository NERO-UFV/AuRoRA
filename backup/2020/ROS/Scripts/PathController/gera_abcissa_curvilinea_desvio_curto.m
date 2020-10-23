function s_desvio = gera_abcissa_curvilinea_desvio_curto(pontoInicial,pontoFinal,obs_ativo,inc_desvio)

% Acha o ângulo entre o arco entre o obstáculo e os pontos
% iniciais e finais do novo caminho
ang_obs_inicio = atan2(pontoInicial(2,1) - obs_ativo(2),pontoInicial(1,1) - obs_ativo(1))*180/pi;
ang_obs_final = atan2(pontoFinal(2,1) - obs_ativo(2),pontoFinal(1,1) - obs_ativo(1))*180/pi;

% As duas condições seguintes mudam os intervalos dos angulos de [-180,180]
% para [0,360], para que a comparação de qual arco é menor seja feita
if ang_obs_inicio < 0
    ang_obs_inicio_positivo = ang_obs_inicio + 360;
else
    ang_obs_inicio_positivo = ang_obs_inicio;
end

if ang_obs_final < 0
    ang_obs_final_positivo = ang_obs_final + 360;
else
    ang_obs_final_positivo = ang_obs_final;
end

% Esta condição define para qual lado o incremento deve crescer (+ ou -),
% dependendo do arco menor e se o angulo inicial é maior ou menor que o
% final
if abs(ang_obs_inicio_positivo - ang_obs_final_positivo) < 180
    if ang_obs_inicio_positivo > ang_obs_final_positivo
        inc_desvio = -abs(inc_desvio);
    else
        inc_desvio = abs(inc_desvio);
    end
else
    if ang_obs_inicio_positivo > ang_obs_final_positivo
        inc_desvio = abs(inc_desvio);
    else
        inc_desvio = -abs(inc_desvio);
    end
end

% Cuida da descontinuidade em [180,-180]
if inc_desvio > 0 && ang_obs_inicio > 0 && ang_obs_final < 0
    ang_obs_final = ang_obs_final_positivo;
end

if inc_desvio < 0 && ang_obs_inicio < 0 && ang_obs_final > 0
    ang_obs_inicio = ang_obs_inicio_positivo;
end

% Define a abcissa curvilínea do desvio
s_desvio = ang_obs_inicio:inc_desvio:ang_obs_final;