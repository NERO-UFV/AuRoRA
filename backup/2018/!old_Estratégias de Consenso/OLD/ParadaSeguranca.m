
if strcmp(bot.String, 'Parar')
    STOP = 1;
    bot.String = 'Rodar';
    disp('Parado')
else
    STOP = 0;
    bot.String = 'Parar';
    disp('Rodando')
end

while STOP

    P.pSC.Ur =  [0; 0];
    P.mEnviarSinaisControle
    drawnow
    pause(1)
end