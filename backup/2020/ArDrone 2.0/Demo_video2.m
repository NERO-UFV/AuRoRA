% testar depois:
% frame_rate = 30;
%  ffplay_command=sprintf('ffplay  -infbuf -f h264 -i http://192.168.1.1:5555 -framerate %d',frame_rate);
% system(ffplay_command);
% ffplay commands:
% -an = disable audio ; -fs = start in fullscreen ; 
% -exitonkeydown = exit if any key is pressed; 

%
% % Capturar dados da camera 
% %% Achei num blog ....
% % http://ardrone-ailab-u-tokyo.blogspot.com.br/2012/07/132-receive-and-decode-video-stream.html
% % AR.Drone sends data as series of 32-bit word, and each 8 bits
% % are aligned reverse ordering (little endiannes). 
% %
% % Each GOB header (for GOB#1~14 ) starts in new 32 bit array,
% % so there are many zeros inserted between the last bit of V0
% % block content and the next GOB header.
% % The offset for chroma value is 128, but that was not the case for AR.
% % Drone and we used 80
% %
% % https://en.wikipedia.org/wiki/YUV#Y.27UV444_to_RGB888_conversion
% %
% %%
% 
% %% Data Stream Format (SDK2 p60)
% % HEADER  : PaVE - Parrot Video Encapsulation
% %
% % 01. uint8_t   signature[4];          / "PaVE" - used to identify the start of frame */
% % 02. uint8_t   version;               / Version code */
% % 03. uint8_t   video_codec;           / Codec of the following frame */
% % 04. uint16_t  header_size;           / Size of the parrot_video_encapsulation_t*/
% % 05. uint32_t  payload_size;          / Amount of data following this PaVE */
% % 06. uint16_t  encoded_stream_width;  / ex: 640 */
% % 07. uint16_t  encoded_stream_height; / ex: 368 */
% % 08. uint16_t  display_width;         / ex: 640 */
% % 09. uint16_t  display_height;        / ex: 360 */
% % 10. uint32_t  frame_number;          / Frame position inside the current stream*/
% % 11. uint32_t  timestamp;             / In milliseconds */
% % - uint8_t   total_chuncks;         / Number of UDP packets containing the current   decodable payload - currently unused */
% % - uint8_t   chunck_index ;         / Position of the packet - first chunk is #0- currenty unused*/
% % 12. uint8_t   frame_type;            / I-frame, P-frame - parrot_video_encapsulation_frametypes_t */
% % 13. uint8_t   control;               / Special commands like end-of-stream or advertised frames */
% % 14. uint32_t  stream_byte_position_lw; / Byte position of the current payload in the encoded stream - lower 32-bit word */
% % 15. uint32_t  stream_byte_position_uw; / Byte position of the current payload in the encoded stream - upper 32-bit word */
% % 16. uint16_t  stream_id;               / This ID indentifies packets that should be recorded together */
% % 17. uint8_t   total_slices;            / number of slices composing the current frame */
% % 18. uint8_t   slice_index ;            / position of the current slice in the frame*/
% % 19. uint8_t   header1_size;            / H.264 only : size of SPS inside payload - no SPS present if value is zero */
% % 20. uint8_t   header2_size;            / H.264 only : size of PPS inside payload - no PPS present if value is zero */
% % - uint8_t   reserved2[2];              / Padding to align on 48 bytes */
% % 21 uint32_t  advertised_size;          / Size of frames announced as advertised frames */
% % - uint8_t   reserved3[12];             / Padding to align on 64 bytes */
% % 
% % Glossary:
% %  PPS - Picture Parameter Set 
% %  SPS - Sequence Parameter Set
% 
% 
% 
close all
clear
clc
%%
% % Close all opened ports
% try
%     fclose(instrfindall);
% catch
% end
% 
% % 
% addpath(genpath(pwd))
% 
% A = ArDrone;  
% % A.rConnect;
% 
% demoTime = 5;         %  simulation time
% Size = 2^20;  %2^15;   % buffer size
% 
% %% Open video port (tcpip 5555)
% 
% try
%     dronePort = tcpip('192.168.1.1', 5555); % in_
%     set(dronePort,'Timeout',5);
%     set(dronePort,'InputBufferSize',Size);
%     
%     fopen(dronePort);
% 
% catch 
% 
%     dronePort = -1;
%     
%     try
%     fclose(dronePort);     
% 
%     catch
%     end
% end
% 
% if dronePort == -1% || localPort == -1
%     disp('SetVideoReceiver failed...')
%     return
% end
% 
% %% Read data
% 
% SequenceNumber = tic;
% 
% t_0 = clock;
% 
% k = 1;              % índice para salvar dados dos sensores
% 
% while etime(clock,t_0) < demoTime
%           
%         if(get( dronePort,'BytesAvailable' )>0)
%           videostream = fread(dronePort);
%           vid{k} = videostream;
%          
%           k = k+1;
%          
%         else           
%             disp('No data');
%         end      
% end
%%
global Q

load vid6;
videostream = vid{2};
%%  Data Storage
% Get all frame headers

% tic
Frame_headers= header_finder(videostream);

% Find I frames
[indexI,frameI] = Iframe_finder(Frame_headers,videostream);
    
% Draw pictures
for k = 1:2%length(frameI)
    figure;
    imshow(uint8((frameI{k})));
end

% toc
%% Close all ports
% fclose(instrfindall);
% fclose(dronePort);

%%

% A.rDisconnect
