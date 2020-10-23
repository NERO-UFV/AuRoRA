function [ U, gamma, fe ] = ControladorPosicaoComDesvioTangencial( P, medidas, dobs, Xponto, k1, k2, gamma, fe, seguir_caminho)
%POSICAOORIENTACAONULA Summary of this function goes here
%   Detailed explanation goes here
    psi = P.pPos.X(6);
    k = [cos(psi), -P.pPar.a*sin(psi); sin(psi), P.pPar.a*cos(psi)];
    [dmin,pos] = min(medidas(2,:));
    beta = medidas(1,pos);
    [Xt,gamma] = VerificaDesvioTangencial(gamma,fe,dobs,dmin,beta,P.pPos.X,P.pPos.Xd);
    
    P.pPos.Xtil = Xt - P.pPos.X;
    fx = k1*tanh(k2 * P.pPos.Xtil(1:2));
    
    if isequal(Xt,P.pPos.Xd) && seguir_caminho
        % Sem desvio
        U = k\(P.pPos.Xd(7:8)+fx);
    else
        U = k\(Xponto+fx);
    end
end

