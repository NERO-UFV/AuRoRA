classdef FormacaoTriangular < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        pPos;
        pSeq;
        pID;
        pPonderacao;
        
        
    end
    
    methods
        function obj = FormacaoTriangular
            mInit(obj);            
        end
        mTransDir(obj,Robo);
        mTransDirDes(obj,Robo);
        mTransInv(obj,Robo);
        mIdentSeq(obj,X);
        mJacobianoDir(obj,Robo);
        mJacobianoInv(obj,Robo);
        mDefineID(obj,ID);
        mTransInvTeste(obj, FiguraDesejada);

    end
    
end

