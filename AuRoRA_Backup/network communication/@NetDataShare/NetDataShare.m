classdef NetDataShare < handle
    properties
        pIP
        pUDP
        pMSG
    end
    
    methods
        function obj = NetDataShare
            mGetBroadcastIP(obj);
            mSetUDP(obj);
            
            obj.pMSG.getFrom = [];
        end
        
        mGetBroadcastIP(obj);
        mSetUDP(obj,str,ID);
        mSendMsg(obj,robot,currentTime);
        mReceiveMsg(obj);       
        
    end
end