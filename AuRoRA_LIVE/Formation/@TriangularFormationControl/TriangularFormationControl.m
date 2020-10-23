classdef TriangularFormationControl < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        pPos;
        pPar;
        pSC;
        pSeq;
        pID;
        pPonderacao;
        pRobots;
    end
    
    methods
        function obj = TriangularFormationControl
            mInit(obj);            
        end
        mDirTrans(obj,Robo);
        mDirTransDesired(obj,Robo);
        mInvTrans(obj,Robo);
        mIdentSeq(obj,X);
        mJacobianoDir(obj,Robo);
        mJacobianoInv(obj,Robo);
        mDefineID(obj,ID);
        mTransInvTeste(obj, FiguraDesejada);
        mTriangularFormationControl(obj,T_INTEG);
        mCSweighting(~,TF,P,method);
    end
    
end

