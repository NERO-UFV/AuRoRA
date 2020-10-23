function mGetBroadcastIP(obj)
%   Return broadcast address of local network
address = java.net.InetAddress.getLocalHost;
IPaddress = char(address.getHostAddress);
A = sscanf(IPaddress,'%d.%d.%d.%d;');
obj.pIP.broadcast = sprintf('%d.%d.%d.%d',A(1:3),255);

end