fprintf(A.pCom.controlChannel, ...
    sprintf('AT*CONFIG=%d,"general:navdata_demo","FALSE"\r',A.pCom.SequenceNumber)); %TRUE/FALSE
A.pCom.SequenceNumber = A.pCom.SequenceNumber+1;
fread(A.pCom.stateChannel,299,'uint8')
%%
    fprintf(A.pCom.controlChannel, ...
        sprintf('AT*CONFIG=%d,"general:navdata_options","105971713"\r',A.pCom.SequenceNumber)); 
    A.pCom.SequenceNumber = A.pCom.SequenceNumber+1;

% sprintf("AT*LED=%d,%d,0,%d\r", ++seq, id, *(int*)(&freq), duration);
%%

string = sprintf('AT*CONFIG=%d,"leds:leds_anim","2,1073741824,5"\r',A.pCom.SequenceNumber);
fprintf(A.pCom.controlChannel,string); %TRUE/FALSE
A.pCom.SequenceNumber = A.pCom.SequenceNumber+1;
%%


string = sprintf('AT*LED=1,3,1056964608,4\r');
fprintf(A.pCom.controlChannel,string); %TRUE/FALSE
A.pCom.SequenceNumber = A.pCom.SequenceNumber+1;
%%


string = sprintf('AT*CONFIG=%d,"general:navdata_options","6"\r',A.pCom.SequenceNumber);
fprintf(A.pCom.controlChannel,string); %TRUE/FALSE
A.pCom.SequenceNumber = A.pCom.SequenceNumber+1;
A.rGetStatusRawData;
disp(A.pCom.nav_data')
%%
 A.rGetStatusRawData
disp(A.pCom.nav_data')



% AR_NAV_CONFIG  = sprintf('AT*CONFIG=2,\"general:navdata_demo\",\"FALSE\"\r') %TRUE/FALSE