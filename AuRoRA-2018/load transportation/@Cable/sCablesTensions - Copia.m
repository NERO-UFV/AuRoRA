-function mCalcTensaoCabos(obj,drone,cabo)


% Tensão aplicada em função do comprimento do cabo
for nv = 1:size(drone,2)
    cabo{nv}.pPos.Xa = cabo{nv}.pPos.X;
    cabo{nv}.pPos.X(4) = norm(drone{nv}.pPos.X(1:3)-obj.pPos.X(1:3));
    cabo{nv}.pPos.X(2) = abs(atan2(drone{nv}.pPos.X(1)-obj.pPos.X(1),drone{nv}.pPos.X(3)-obj.pPos.X(3)));
end

% Apenas para o caso de dois Veículos
if nv == 2
    if cabo{1}.pPos.X(4) < cabo{1}.pPar.l
        cabo{1}.pPos.X(3) = 0;
    else
        cabo{1}.pPos.X(3) = obj.pPar.m*obj.pPar.g*sin(cabo{2}.pPos.X(2))/(cos(obj.pPos.alpha)*sin(cabo{1}.pPos.X(2)+cabo{2}.pPos.X(2))) + 0.1*(cabo{1}.pPos.X(4) - cabo{1}.pPar.l) + 1*(cabo{1}.pPos.X(4)-cabo{1}.pPos.Xa(4))/obj.pPar.Ts;
    end
    
    if cabo{2}.pPos.X(4) < cabo{2}.pPar.l
        cabo{2}.pPos.X(3) = 0;
    else
        cabo{2}.pPos.X(3) = obj.pPar.m*obj.pPar.g*sin(cabo{1}.pPos.X(2))/(cos(obj.pPos.alpha)*sin(cabo{1}.pPos.X(2)+cabo{2}.pPos.X(2))) + 0.1*(cabo{2}.pPos.X(4) - cabo{2}.pPar.l) + 1*(cabo{2}.pPos.X(4)-cabo{2}.pPos.Xa(4))/drone{1}.pTempo.Ts;
    end
    
    % Integração numérica da posição da carga
    obj.pPos.Xa = obj.pPos.X;
    obj.pPos.X(1) = obj.pPos.Xa(1) + obj.pPar.Ts*(obj.pPos.Xa(4) + obj.pPar.Ts*(cabo{1}.pPos.X(3)*sin(cabo{1}.pPos.X(2))*cos(obj.pPos.alpha) - cabo{2}.pPos.X(3)*sin(cabo{2}.pPos.X(2)*cos(obj.pPos.alpha)) )/obj.pPar.m);
    obj.pPos.X(4) = (obj.pPos.X(1)-obj.pPos.Xa(1))/obj.pPar.Ts;
    
    % Comportamento do ângulo alpha
    obj.pPos.ddalpha = -(cabo{1}.pPos.X(3)+cabo{2}.pPos.X(3))*sin(obj.pPos.alpha)+1/(obj.pPar.Iyy+obj.pPar.m*obj.pPar.l^2)*(obj.pPar.m*obj.pPar.l*(... % O sinal do (cabo{1}.pPos.X(3)+cabo{2}.pPos.X(3))*sin(obj.pPos.alpha) causa intriga
        cos(obj.pPos.alpha)*( drone{1}.pPos.dX(8) - 2*obj.pPos.dalpha*drone{1}.pPos.X(9)) + ...
        sin(obj.pPos.alpha)*(-drone{1}.pPos.dX(9) - 2*obj.pPos.dalpha*drone{1}.pPos.X(8) - obj.pPar.g)));
    obj.pPos.dalpha = obj.pPos.dalpha + obj.pPos.ddalpha*obj.pPar.Ts;
    obj.pPos.alpha = obj.pPos.alpha + obj.pPos.dalpha*obj.pPar.Ts;
    
    obj.pPos.X(2) = tan(obj.pPos.alpha)*(drone{1}.pPos.X(1)-obj.pPos.X(1)) + drone{1}.pPos.X(2);
    obj.pPos.X(5) = (obj.pPos.X(2)-obj.pPos.Xa(2))/obj.pPar.Ts;
    
    obj.pPos.X(3) = obj.pPos.Xa(3) + obj.pPar.Ts*(obj.pPos.Xa(6) + obj.pPar.Ts*(cabo{1}.pPos.X(3)*cos(cabo{1}.pPos.X(2))*cos(obj.pPos.alpha) + cabo{2}.pPos.X(3)*cos(cabo{2}.pPos.X(2))*cos(obj.pPos.alpha) - obj.pPar.m*obj.pPar.g)/obj.pPar.m);
    if obj.pPos.X(3) < 0; obj.pPos.X(3) = 0;end
    obj.pPos.X(6) = (obj.pPos.X(3)-obj.pPos.Xa(3))/obj.pPar.Ts;
end
if nv == 3
    
    
end


end
