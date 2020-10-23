function mConectarMobileSim(obj)

obj.pFlag.Connected = 1;
if isempty(obj.pPort) || obj.pPort == 0
obj.pPort = 8101;
end


eval(['aria_init -rh 127.0.0.1:' num2str(obj.pPort ) ])
% aria_init;
arrobot_connect;

end
