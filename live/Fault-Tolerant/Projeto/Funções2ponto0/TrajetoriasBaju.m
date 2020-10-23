function TrajetoriasBaju(trif,QdA,tatual,etapa)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    T_MAX = 30; %% USAR EVALIN

    switch etapa
        case 1
            % FORMAÇÃO 1
            trif{1}.pPos.Qd = QdA{1};
            trif{1}.pPos.Qd(2) = QdA{1}(2)+(1.3-QdA{1}(2))*tatual/T_MAX;
            % FORMAÇÃO 2
            trif{2}.pPos.Qd = QdA{2};
            trif{2}.pPos.Qd(2) = QdA{2}(2)+(1.3-QdA{2}(2))*tatual/T_MAX;
        case 2
            % FORMAÇÃO 1
            trif{1}.pPos.Qd = QdA{1};
            trif{1}.pPos.Qd(1) = QdA{1}(1)+(-.8-QdA{1}(1))*tatual/T_MAX;
            trif{1}.pPos.Qd(2) = QdA{1}(2)+(-1.4-QdA{1}(2))*tatual/T_MAX;
            
        case 3
        case 4
        case 5
        case 6
        case 7
        case 8
    end

end

