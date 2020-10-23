function mEnviarSinaisControle(obj)

% Verificar se joystick conectado
if obj.pSC.Joystick.OK == 1
    obj.pSC.Joystick.Botoes = button(obj.pSC.Joystick.J);
    obj.pSC.Joystick.Eixos  = axis(obj.pSC.Joystick.J);
    
    for ii = 1:length(obj.pSC.Joystick.Eixos)
        if abs(obj.pSC.Joystick.Eixos(ii)) < 0.3
            obj.pSC.Joystick.Eixos(ii) = 0;
        end
    end        
    
    if obj.pSC.Joystick.Botoes(5) == 1
        % Prioridade para comandos do Joystick
        switch upper(obj.pSC.Joystick.Modelo)
            case 'XBOX'
                obj.pSC.Joystick.Ar = [0.1 0.1 0.1 0.4]'.*obj.pSC.Joystick.Eixos([5 4 1 2])'.*[-1 1 -1 -1]';
            case 'PS'
                obj.pSC.Joystick.Ar = [0.1 0.1 0.1 0.4]'.*obj.pSC.Joystick.Eixos([4 3 2 1])'.*[-1 1 -1 1]';
        end
    end    
end

if obj.pStatus.Conectado
    % Modo Experimento
    arrobot_setvel(obj.pSC.Ur(1)*1000);
    arrobot_setrotvel(obj.pSC.Ur(2)/pi*180);    
else
    % Modo simulação
    obj.pSC.U = obj.pSC.Ur;
    obj.mDefinirPostura;
end

end
