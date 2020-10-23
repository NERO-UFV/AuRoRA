function rGetIp(drone)
% Função rGetIp(drone)
% Identifica automaticamente o ip do drone conectado ao Wi-Fi
% Requisitos: - Sistema Operacional Windows
%             - Setar o ID do drone ao iniciar a classe. 
%             Ex: A{1} = ArDrone(1); A{2} = ArDrone(2); etc
% Sempre configurar o ip do drone com início '192.x.x.x'

% Armazenando o texto do comando do Windows ipconfig
[~, ipText] = system('ipconfig');

% Transformando o texto armazenado em celulas strings
ipText = split(ipText);

% Iniciando as variaveis
ipDrone = [];       % Responsavel por armazenar os ip's dos drones
flagWiFi = 0;       % Flag ativada após ler a string 'Wi-Fi'
flagGateway = 0;    % Flag ativada após ler a string 'Gateway'

% For que irá iterar sobre todas as celulas de ipText
for ii = 1:size(ipText,1)
try
% If que identifica quando a string da célula é igual à 'Wi-Fi'
if sum(ipText{ii}(1:5) ~= 'Wi-Fi') == 0
flagWiFi = 1;       % Ativa a flag do 'Wi-Fi'
end
catch
end
try
% If que identifica quando a string da célula é igual à 'Gateway'
if flagWiFi == 1 && sum(ipText{ii} ~= 'Gateway') == 0  
flagGateway = 1;    % Ativa a flag do 'Gateway'
flagWiFi = 0;       % Desativa a flag do 'Wi-Fi'
end
catch
end
try
% If que identifica quando o inicio da string da célula é igual à '192'
if flagGateway ~= 0 && sum(ipText{ii}(1:3) ~= '192') == 0
ipDrone{end+1} = ipText{ii};    % Armazena o ip encontrado
flagGateway = 0;                % Desativa a flag 'Gateway'
end
catch
end
end
try
% Armazena o ip no drone de acordo com seu ID
drone.pPar.ip = ipDrone{drone.pID};
catch
end
end