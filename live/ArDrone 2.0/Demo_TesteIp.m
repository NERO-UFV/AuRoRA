function rGetIp(drone)
[~, str] = system('ipconfig');
str = split(str);

ipDrone = [];
flagGateway = 0;
flagWiFi = 0;

for ii = 1:size(str,1)
try
if sum(str{ii}(1:5) ~= 'Wi-Fi') == 0
flagWiFi = 1;
end
catch
end
try
if flagWiFi == 1 && sum(str{ii} ~= 'Gateway') == 0
flagGateway = 1;
flagWiFi = 0;
end
catch
end
try
if flagGateway ~= 0 && sum(str{ii}(1:3) ~= '192') == 0
ipDrone{end+1} = str{ii};
flagGateway = 0;
end
catch
end
end

drone.pPar.ip = ipDrone{drone.
end