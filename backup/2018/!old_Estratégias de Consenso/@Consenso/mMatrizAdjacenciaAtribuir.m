function mMatrizAdjacenciaAtribuir(obj,Madj)
obj.pVF.Madj = Madj;
obj.pVF.nID = length(obj.pVF.Madj);

obj.pVF.Ei  = nan(3,obj.pVF.nID);
obj.pVF.dEi = zeros(3,obj.pVF.nID);
obj.pVF.X = zeros(12,obj.pVF.nID);

obj.pVF.XF   = zeros(3,obj.pVF.nID); 

end
