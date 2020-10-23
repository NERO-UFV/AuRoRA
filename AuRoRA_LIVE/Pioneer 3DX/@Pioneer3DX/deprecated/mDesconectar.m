function mDesconectar(obj)

obj.pStatus.Conectado = 0;

for ii = 1:3
    arrobot_disconnect;
    pause(1)
    aria_shutdown;
    pause(1)
end
end
