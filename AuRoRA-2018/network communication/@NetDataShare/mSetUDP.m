function mSetUDP(obj)
% Set and open UDP communication
obj.pUDP = udp(obj.pIP.broadcast);
obj.pUDP.LocalPort  = 9090;
obj.pUDP.RemotePort = 9090;

% Parameters
obj.pUDP.Timeout = 1;
obj.pUDP.InputBufferSize = 2^12;

% obj.pUDP.EnablePortSharing = 'on'; % <ativado por Marcos 16/07/18 para teste optitrack>

fopen(obj.pUDP);
end