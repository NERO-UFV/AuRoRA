function mCSweighting(~,TF,P)
    switch TF{1}.pPonderacao
        % Tradicional
        case 0
            for ii=1:length(TF)
                P{ii}.pPos.Xd(1:2) = TF{ii}.pPos.Xr(1:2);  % Posição desejada
                P{ii}.pPos.Xd(7:8) = TF{ii}.pPos.dXr(1:2); % Velocidade desejada

                P{ii+1}.pPos.Xd(1:2) = TF{ii}.pPos.Xr(4:5);
                P{ii+1}.pPos.Xd(7:8) = TF{ii}.pPos.dXr(4:5);

                P{ii+2}.pPos.Xd(1:2) = TF{ii}.pPos.Xr(7:8);
                P{ii+2}.pPos.Xd(7:8) = TF{ii}.pPos.dXr(7:8);
            end
            
        % Media Simples
        case 1
            for ii=1:length(TF)
                P{TF{ii}.pID}.pPos.mXd(1:2,TF{ii}.pID)   = TF{ii}.pPos.Xr(1:2);
                P{TF{ii}.pID+1}.pPos.mXd(1:2,TF{ii}.pID) = TF{ii}.pPos.Xr(4:5);
                P{TF{ii}.pID+2}.pPos.mXd(1:2,TF{ii}.pID) = TF{ii}.pPos.Xr(7:8);
                
                P{TF{ii}.pID}.pPos.mdXd(1:2,TF{ii}.pID)   = TF{ii}.pPos.dXr(1:2);
                P{TF{ii}.pID+1}.pPos.mdXd(1:2,TF{ii}.pID) = TF{ii}.pPos.dXr(4:5);
                P{TF{ii}.pID+2}.pPos.mdXd(1:2,TF{ii}.pID) = TF{ii}.pPos.dXr(7:8);
            end

            for ii=1:length(P)
                P{ii}.pPos.Xd(1:2) = sum(P{ii}.pPos.mXd,2)./length(P{ii}.pFormation);
                P{ii}.pPos.Xd(7:8) = sum(P{ii}.pPos.mdXd,2)./length(P{ii}.pFormation);
            end
        
        % Média Ponderada
        case 2
            for ii=1:length(TF)
                P{TF{ii}.pID}.pPos.mXd(1:2,TF{ii}.pID)   = TF{ii}.pPos.Xr(1:2);
                P{TF{ii}.pID+1}.pPos.mXd(1:2,TF{ii}.pID) = TF{ii}.pPos.Xr(4:5);
                P{TF{ii}.pID+2}.pPos.mXd(1:2,TF{ii}.pID) = TF{ii}.pPos.Xr(7:8);
                
                P{TF{ii}.pID}.pPos.mdXd(1:2,TF{ii}.pID)   = TF{ii}.pPos.dXr(1:2);
                P{TF{ii}.pID+1}.pPos.mdXd(1:2,TF{ii}.pID) = TF{ii}.pPos.dXr(4:5);
                P{TF{ii}.pID+2}.pPos.mdXd(1:2,TF{ii}.pID) = TF{ii}.pPos.dXr(7:8);
            end
            mQj = zeros(1, length(TF));
            MatrizPonderacao = zeros(length(P), length(TF));
            for ii=1:length(TF)
                mQj(1,ii) = norm(TF{ii}.pPos.Qtil);
                MatrizPonderacao(ii, ii) = 1;
                MatrizPonderacao(ii+1, ii) = 1;
                MatrizPonderacao(ii+2, ii) = 1;
            end
            for ii=1:length(P)
                somaDenominador = 0;
                somaNumerador = zeros(4,1);
                for jj=1:length(TF)
                    somaDenominador = somaDenominador + MatrizPonderacao(ii,jj)*mQj(1,jj);
                    somaNumerador = somaNumerador + MatrizPonderacao(ii,jj)*mQj(1,jj)*[P{ii}.pPos.mXd(:,jj);P{ii}.pPos.mdXd(:,jj)];
                end
                P{ii}.pPos.Xd(1:2) = somaNumerador(1:2)./somaDenominador;
                P{ii}.pPos.Xd(7:8) = somaNumerador(3:4)./somaDenominador;
            end
    end
end