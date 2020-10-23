function mConectar(obj)

obj.pStatus.Conectado = 1;
% obj.pPort = 8100+obj.pID;

% eval(['aria_init -rh 127.0.0.1:' num2str(obj.pPort ) ])
aria_init;
arrobot_connect;

end
