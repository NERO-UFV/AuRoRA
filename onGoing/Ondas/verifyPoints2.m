function [leftPoints, rightPoints] = verifyPoints2(pa, pb, pointsMatrix)

    % pa e pb sao os pontos que determinam a reta e pointsMatrix é 
    % a matriz com os pontos com os vertices do objeto que se deseja
    % saber a posicao em relacao a reta. pa e pb são vetores 1x2 e 
    % pointsMatrix é uma matriz 3xn, sendo n o numero de vertices do
    % objeto.
    
    leftPoints = [];
    rightPoints = [];
    
    for i = 1:length(pointsMatrix) 
    
        % Transformacao Linear: (x, y, z) -> (x, z):

        pc = [pointsMatrix(1, i), pointsMatrix(3, i)];
        
        % Inicializando vetores que irao compor o calculo do determinante:

        vetorOnes = [1; 1; 1];
        matrixPoints = [pa; pb; pc];
        determinant = det([matrixPoints, vetorOnes]);
        
        % Condicional para verificacao da posicao do ponto em relacao a
        % reta:

        if determinant > 0 % esquerda
            leftPoints = [leftPoints pointsMatrix(1:3, i)]; 
        elseif determinant < 0 % direita
            rightPoints = [rightPoints pointsMatrix(1:3, i)];
        end

    end
    
%     disp('Pontos à Esquerda:')
%     disp(leftPoints)
%     disp('')
%     disp('Pontos à Direita:')
%     disp(rightPoints)
    
end