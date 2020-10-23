function mDefinirPostura(obj,modelo)

% mDefinirPostura determina a postura do Pioneer baseado em seu modelo

if nargin < 2
    modelo = 'C';
end

switch modelo
    case 'C' % Modelo Cinemático plano 2D
        
        K = [cos(obj.pPos.X(6)) -obj.pPar.a*sin(obj.pPos.X(6)); sin(obj.pPos.X(6)) obj.pPar.a*cos(obj.pPos.X(6)); 0 1];
        
        % Postura do robô em seu sistema de referência
        % Posição
        obj.pPos.X([1 2 6]) = obj.pPos.X([1 2 6]) + K*obj.pSC.U*obj.pTempo.Ts;
        
        obj.pPos.X([7 8 12]) = K*obj.pSC.U;


        % limitação de quadrante do ângulo
    for ii = 4:6
        if abs(obj.pPos.X(ii)) > pi
            if obj.pPos.X(ii) < 0
                obj.pPos.X(ii) = obj.pPos.X(ii) + 2*pi;
            else
                obj.pPos.X(ii) = obj.pPos.X(ii) - 2*pi;
            end
        end
    end
end

end