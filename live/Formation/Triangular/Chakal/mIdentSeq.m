function mIdentSeq(obj,Robo )
%   Identificação da sequência da formação triangular
k = cross([Robo{obj.pID+1}.pPos.X(1)-Robo{obj.pID}.pPos.X(1), Robo{obj.pID+1}.pPos.X(2)-Robo{obj.pID}.pPos.X(2), 0],[Robo{obj.pID+2}.pPos.X(1)-Robo{obj.pID}.pPos.X(1), Robo{obj.pID+2}.pPos.X(2)-Robo{obj.pID}.pPos.X(2), 0]);

%k = cross([Robo(1,2)-Robo(1,1), Robo(2,2)-Robo(2,1), 0],[Robo(1,3)-Robo(1,1), Robo(2,3)-Robo(2,1), 0]);


if k(3)>0
    obj.pSeq = 1;
else
    obj.pSeq = -1;   
   % Robo{obj.pID+2}.pPos.Robo{obj.pID}.pPos.X(2) = -Robo{obj.pID+2}.pPos.Robo{obj.pID}.pPos.X(2);
end

end

