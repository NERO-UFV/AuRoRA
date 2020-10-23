classdef Consenso < handle
    % In a methods block, set the method attributes
    % and add the function signature
    properties
        pVF % Indica variáveis de formação
        pVC % Indica variáveis de comunicação
        
        
    end
    
    methods
        function obj = Consenso
            mInit(obj);
        end
        
        mInit(obj);
        
        % Atribuir matriz de adjacência
        mMatrizAdjacenciaAtribuir(obj,Madj);
        
        % Conjunto de funções para interação em consenso
        mConexoesCriar(obj);                         % Abertura de portas para buscar dados 
                                                     % dos demais robôs.
        mEstadoInformacaoPopular(obj,robo);          % Adquirir dados dos robôs através da rede
        robo = mEstadoInformacaoAtualizar(obj,robo); % Atualizar nos robôs os dados adquiridos por consenso
        
        
        
    end
end