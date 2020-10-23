close all
clear all
clc

%Habilitando comunicação Ethernet
t = tcpip ('169.254.146.116','remoteport',2112,'localport',2112);
pause(0.5)
disp('---------------Rodando...Habilitando comunicação Ethernet')
disp('---------------Quem é t?')
pause(0.5)
t

%Habilitando Terminador
set(t,'terminator',3)
pause(0.5)
disp('---------------Rodando...Habilitando Terminador')
disp('---------------Quem é t?')
pause(0.5)
t

set(t, 'InputBufferSize', 3000)
get(t, 'InputBufferSize');
pause(0.5)
set(t, 'OutputBufferSize', 3000)
get(t, 'OutputBufferSize');
set(t, 'ReadAsyncMode', 'continuous')
set(t, 'Timeout', 60)

%Abertura de Porta(Comunicação PC Sick)
disp('---------------Abertura de Porta(Comunicação PC >>> Sick)')
pause(0.5)
fopen(t)
pause(0.5)
disp('---------------Rodando')
t

%Pedido de Acesso ao Dispositivo
fwrite(t,[2 'sMN SetAccessMode 03 F4724744' 3])
disp('---------------Acesso Cliente')
pause(0.5)
disp('---------------Pedido de Acesso ao Dispositivo')
pause (0.5)
disp('---------------Resposta = 1, Tudo OK!')
fscanf(t)

%Obtenção de qual frequência e qual resolução de escaneamento
fwrite(t,[2 'sRN LMPscancfg' 3])
pause(0.5)
disp('---------------Qual frequência e resolução de escaneamento?')

% Resposta do requisição
fscanf(t)
%frequencia__50Hz = 1388h (5000d)
%reservado = 1
%resolução angular = 0,5° 1388h (5000d)
% angulo de inicio e fin do escanemento = FFF92230h..225510h = –450000d..+2250000d)

%Obtendo status do dispositivo
fwrite(t,[2 'sRN LCMstate' 3]);
pause(0.5)
disp('---------------Qual status do dispositivo?')
pause(0.5)
disp('---------------Resposta = 0, Tudo OK!')
fscanf(t)

%Real faixa de saida de dados
fwrite(t,[2 'sRN LMPoutputRange' 3])
pause(0.5)
%resposta do status
disp('---------------Qual a real faixa de saida de dados?')
pause(0.5)
fscanf(t)
%Remissão = 1 (com remissão)
%resolução angular = 0,5° 1388h (5000d)
%angulo de início  = 0 (0º)
%angulo de término =  1B7740 (180º)

%Situaçao do Dispositivo
fwrite(t,[2 'sMN LMCstartmeas' 3])
disp('---------------Qual a situação do dispositivo?')
pause(0.5)
disp('---------------Escaneamento Parado')
pause(0.5)
disp('---------------Resposta = 0, Tudo OK!')
fscanf(t)
pause(0.5)
disp('---------------Qual a situação do dispositivo?')
pause(0.5)
fwrite(t,[2 'sMN LMCstartmeas 1' 3])
disp('---------------Início do Escaneamento')
pause(0.5)
disp('---------------Resposta = 0, Tudo OK!')
fscanf(t)
disp('---------------Início de rotação do espelho')
pause (10)
disp('-------------------------PRESSIONE TECLA PAUSE PARA CONTINUAR-----------------------')

%Acionando com um clique
pause
fwrite(t,[2 'sEN LMDscandata 1' 3])
fscanf(t)

disp('---------------Gravando')

% Gravação dos dados
t.RecordDetail = 'verbose';
t.RecordName = 'Dados_originais_vindos_do_SICK.txt';
record(t,'on')
fprintf(t,'*IDN?')
disp('---------------Taíííí o que vc queria')
out = fscanf(t)
 FigHandle = figure('Position', [100, 100, 1200, 600]);
zoom on;
ind = 1;
for n =1:250
    k = n;
    k = fscanf(t)
    try
        ini = strfind(k,'21D ') % buscar inicio dos dados
        
        esp = strfind(k,' ')% encontrar todos os espaços
        med = k(ini(1): esp(end-5)) %vetor de medidas (os valores que me intereçam)
        espmed = strfind(med, ' '); %espaços dentro da medida (dos valore que realmente me intereçam)
        clear valores_do_laser
        for ii = 1:length(espmed)-1
            valores_do_laser(ii) = hex2dec(med(espmed(ii)+1:espmed(ii+1)-1));
        end
        
        %delete(h)
    end
    if length(valores_do_laser) == length(-45:0.5:225)
        %hold on
       %______________________2D__________________________________________
        h1 = plot(valores_do_laser.*cosd(-45:0.5:225),valores_do_laser.*sind(-45:0.5:225),'.g');
        subplot(1,2,1); 
        title('POSSÍVEL 2D DE PLANTAS--270º')
        xlabel('Coordenada X');
        ylabel('Coordenada Y');
        zlabel('Coordenada Z');
        view(90,-90); 
        hold on;
        %______________________3D__________________________________________
        h2 = plot3(valores_do_laser.*cosd(-45:0.5:225),valores_do_laser.*sind(-45:0.5:225),ind*ones(1,length(valores_do_laser)),'.');
        subplot(1,2,2); 
        title('POSSÍVEL 3D DE PLANTAS--270º')
        xlabel('Coordenada X');
        ylabel('Coordenada Y');
        zlabel('Coordenada Z');
        view(90,-90);
        hold on;
    end
    drawnow
    
    C=[valores_do_laser.*cosd(-45:0.5:225);valores_do_laser.*sind(-45:0.5:225);ind*ones(1,length(valores_do_laser))];
    D = C'
        
    X(:,ind)=C(1,:);%----valor de X;
    Y(:,ind)=C(2,:);%----valor de Y;
    Z(:,ind)=C(3,:);%----valor de Z;
    %Z1 = Z*10
    D(1+541*(ind-1):541*ind,1)=X(:,ind);
    D(1+541*(ind-1):541*ind,2)=Y(:,ind);
    D(1+541*(ind-1):541*ind,3)=Z(:,ind);
    %D(1+541*(ind-1):541*ind,3)=Z1(:,ind);
    
    
    ind = ind + 1;
    
    pause (0.000001)
    flushinput(t);
    
end
    
    
    xlswrite('valores_de__X.xls', X, 'teste3__Valores_de_X', 'A1');
    xlswrite('valores_de__Y.xls', Y, 'teste3__Valores_de_Y', 'A1');
    xlswrite('valores_de__Z.xls', Z, 'teste3__Valores_de_Z', 'A1');
    %xlswrite('valores_de__Z.xls', Z1, 'teste3__Valores_de_Z', 'A1');
    %----------------------------------------------------------------------
    

%--------------------------------------------------------------------------

    
%--------------------------------------------------------------------------    
%Parando Gravação
record(t,'off')

%Parando Dispositivo
fwrite(t,[2 'sMN LMCstopmeas 1' 3])
disp('---------------Término do Escaneamento')
pause (0.5)

%Fechando Porta
disp('---------------Fechando Porta')
pause(0.5)
fclose(t)
disp('---------------Porta Fechada')