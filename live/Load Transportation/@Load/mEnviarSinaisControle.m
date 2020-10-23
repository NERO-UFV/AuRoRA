function mEnviarSinaisControle(carga,veiculo)


% Verificar se joystick conectado
if veiculo.pSC.Joystick.OK == 1
    veiculo.pSC.Joystick.Botoes = button(veiculo.pSC.Joystick.J);
    veiculo.pSC.Joystick.Eixos  = axis(veiculo.pSC.Joystick.J);
    
    for ii = 1:length(veiculo.pSC.Joystick.Eixos)
        if abs(veiculo.pSC.Joystick.Eixos(ii)) < 0.3
            veiculo.pSC.Joystick.Eixos(ii) = 0;
        end
    end        
    
    if veiculo.pSC.Joystick.Botoes(5) == 1
        % Prioridade para comandos do Joystick
        switch upper(veiculo.pSC.Joystick.Modelo)
            case 'XBOX'
                veiculo.pSC.Joystick.Ar = [0.1 0.1 0.1 0.4]'.*veiculo.pSC.Joystick.Eixos([5 4 1 2])'.*[-1 1 -1 -1]';
            case 'PS'
                veiculo.pSC.Joystick.Ar = [0.1 0.1 0.1 0.4]'.*veiculo.pSC.Joystick.Eixos([4 3 2 1])'.*[-1 1 -1 1]';
        end
    end
    
    if veiculo.pSC.Joystick.Botoes(6) == 1
        veiculo.pStatus.Abortar = 1;
        if veiculo.pStatus.Conectado
            veiculo.mLand;
        end
    end
end

if veiculo.pStatus.Conectado
    % Modo Experimento
    veiculo.mDrive(veiculo.pSC.Joystick.Ar');    
else
    % Modo simulação
    carga.mDefinirPostura(veiculo);
end
end
