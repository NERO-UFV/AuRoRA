function mEstadoInformacaoPopular(obj,robo)

% Popular os estados de informação do consenso, em função das variáveis
% transmitidas pelos robôs. Todas as variáveis poderão ser populadas,
% entretanto somente serão utilizadas aquelas relacionadas à matriz de
% adjacência

if nargin < 1
    robo = [];
end

if obj.pVC.ConexoesFlag
    for ii = 1:length(obj.pVC.Conexoes)
        [ID, POS, SC, flag] = mConexoesLer(obj.pVC.Conexoes(ii));
        if flag
            obj.pVF.X(:,ID)   = POS;
            obj.pVF.Ei(:,ID)  = POS(1:3);
            obj.pVF.dEi(:,ID) = POS(7:9);
            obj.pVF.SC(:,ID)  = SC;
        end
    end
else
    obj.pVF.X(:,robo.pID)   = robo.pPos.X;
    obj.pVF.Ei(:,robo.pID)  = robo.pPos.X(1:3);
    obj.pVF.dEi(:,robo.pID) = robo.pPos.X(7:9);
    obj.pVF.SC(1:length(robo.pSC.U),robo.pID)  = robo.pSC.U;
end
end

function [ID, POS, SC, flag] = mConexoesLer(Conexao)
flag = 0;
ID = NaN;
POS = NaN(12,1);
SC = NaN(2,1);
while get(Conexao, 'BytesAvailable') && flag ~= 1
    msg = fscanf(Conexao);
    
    if length(msg) > 4 && strcmp(msg(1:4),'$POS')
        flag = -1;
        [A, count ]= sscanf(msg,'$POSX%u:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,SC:%f,%f,%f,%f;');
        if count == 17
            ID = A(1);
            POS = A(2:13);
            SC = A(14:17);
            flag = 1;
            flushinput(Conexao);
        end
    end
end

end
