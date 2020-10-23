% Capturar dados da camera 
%% Achei num blog ....
% http://ardrone-ailab-u-tokyo.blogspot.com.br/2012/07/132-receive-and-decode-video-stream.html
% AR.Drone sends data as series of 32-bit word, and each 8 bits
% are aligned reverse ordering (little endiannes). 
% To interpret these values as binary video stream as explained
% in AR.Drone Developer Guide, convert the address of array,
% bufferedVideoStream to 8-bit array display in the order each 4 
% elements are reverse ordered.
%
% Each GOB header (for GOB#1~14 ) starts in new 32 bit array,
% so there are many zeros inserted between the last bit of V0
% block content and the next GOB header.
% The offset for chroma value is 128, but that was not the case for AR.
% Drone and we used 80
%
% https://en.wikipedia.org/wiki/YUV#Y.27UV444_to_RGB888_conversion
%
%%

%% Data Stream Format (SDK2 p52)
% For each picture: 
%    -----------------------------------------------------------
%   |PICTURE HEADER + DATA BLOCK GROUPS(GOBS) + ENDING CODE(EOS)|
%    -----------------------------------------------------------
%
% PICTURE HEADER: 
%   PSC                - 22 bits     
%   Format             -  2 bits      
%   Resolution         -  3 bits
%   Picture Type       -  3 bits
%   Quant              - [p264:6 bits]|[uvlc:5bits]
%   Frame Number       - 32 bits
% 
%   TOTAL . . . . . .  = 68 / 67 bits 
% 
% GOB:
%   Gobsc              - 22 bits
%   Quant              - [h264:6 bits]|[uvlc:5bits]
%   *Gob               - XXX bits
%
%   *Gob: UVLC macroblock layer
%       MBC            - 1 bit
%       MBDES          - 8 bits
%       MBDIFF         - 2 bits (reserved)
%       Y0             - XXX bits
%       Y1             - XXX bits
%       Y2             - XXX bits
%       Y3             - XXX bits
%       U0             - XXX bits
%       V0             - XXX bits
%
%   *Gob: P264 macroblock INTRA layer
%       luma type      - 1 bit
%       chroma type    - 2 bits
%          >Intra4x4   >>
%            mode(16*xxxbits)//Y0(xxxbits)...//Y15(xxxbits)//CHROMADATA**
%          
%          >Intra16x16 >> 
%            mode(2bits)//dcY(xxxbits)//acY0(xxxbits)//...//Y15(xxxbits)//CHROMADATA**
%  
%   *Gob: P264 macroblock INTER layer
%        partition list     - 3 bits
%        motion vector list - xxx bits 
%        Y0                 - xxx bits
%        ...
%        Y15                - xxx bits
%        CHROMADATA**
% 
%                   ** CHROMADATA
%                        DC U(xxxbits)
%                        AC U0(xxxbits)
%                        ...
%                        AC U3(xxxbits)
%                        DC V(xxxbits)
%                        ACV0(xxxbits)
%                        ...
%                        AC V3(xxxbits)
%
%
%
% EOS(END OF SEQUENCE) - 22 bits
%        
% 
%% Description
%
%      > PSC(picture start code)         - 22 bits
%            -----------------------------------
%           |uvlc = 0000 0000 0000 0000 1 00000 |
%           |p264 = 0000 0000 0000 0001 0 00000 |
%            -----------------------------------  
%      > Format                          - 2 bits
%            ---------------
%           |forbidden = 00 | 
%           |CIF       = 01 |
%           |VGA       = 10 |     
%            ---------------
%      > Resolution                      - 3 bits
%            -------------------------------
%           |forbidden                = 000 | 
%           |for CIF means sub-QCIF   = 001 |
%           |for CIF means QCIF       = 010 |
%           |for CIF means CIF        = 011 |
%           |for CIF means 4-CIF      = 100 |
%           |for CIF menas 16-CIF     = 101 | 
%            -------------------------------
%      > Picture Type                     - 3 bits
%            ----------------------
%           |intra picture   = 000 | 
%           |inter picture   = 001 |
%            ----------------------     
%      > Quant                            - 5/6 bits
%            --------------------------------- 
%           |uvlc codec : 5bits word  >>> 1~30|
%           |p264 codec : 6 bits word >>> 0~63|
%            ---------------------------------
%      > GOBSC(Group of block start code) - 22bits
%            -----------------------------------
%           |uvlc : 0000 0000 0000 0000 1XXX XX |
%           |p264 : 0000 0000 0000 0001 0XXX XX |
%            -----------------------------------
%   Note: A GOBSC is always a byte aligned. The least significant bytes
%   represent the blockline’s number.
%   We can see that PSC means first GOB too. So for the first GOB, 
%   GOB’s header is alway
% 
% 
% 
%       > EOS(End of Sequence)            - 22 bits
%            ----------------------------    
%           |0000 0000 0000 0001 0111 11 |
%            ----------------------------  

close all
clear
clc

% Fecha possíveis portas abertas
try
    fclose(instrfindall);
catch
end

% Adiciona pasta das funções da classe e inicializa portas
addpath(genpath(pwd))

A = ArDrone;  
% A.rConnect;

demoTime = 5; %  tempo de simulação
Size = 2^20; %2^15;   % buffer size

%% Abertura da porta de video (tcpip 5555)

try
    dronePort = tcpip('192.168.1.1', 5555); % in_
    set(dronePort,'Timeout',5);
    set(dronePort,'InputBufferSize',Size);
    
    fopen(dronePort);

catch 

    dronePort = -1;
    
    try
    fclose(dronePort);     

    catch
    end
end

if dronePort == -1% || localPort == -1
    disp('SetVideoReceiver failed...')
    return
end

%%

SequenceNumber = tic;

t_0 = clock;

k = 1;              % índice para salvar dados dos sensores

while etime(clock,t_0) < demoTime
          
        if(get( dronePort,'BytesAvailable' )>0)
          DataReceived = fread(dronePort);
          vid{k} = DataReceived;
         
          k = k+1;
         
        else           
            disp('No data');
        end      
end

%% 
fclose(instrfindall);
fclose(dronePort);

%%

% A.rDisconnect
