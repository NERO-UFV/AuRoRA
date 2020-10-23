%% Data storage function
% Find all the the frame headers start and returns a cell. 
% Each cell element contains a header structure
%
% All Header Start with : "P a V E"
% ASCII code -> decimal:
% P = 80 ; a = 97 ; V = 86 ; E = 69;

function Frame = header_finder(videostream)
n = 1;

% Search for frame header start INDEX = Signature "PaVE"
   for ii = 1:length(videostream)
     
       if videostream(ii)==80 && videostream(ii+1)==97 && ...
           videostream(ii+2)==86 && videostream(ii+3)==69
       
        header_start(n) = ii;
        n = n+1;
        
       end
      
      % should not happen, but...
      if ii == length(videostream) && n==1
          disp('No signature found!');
      end
        
   end
 
% Store data from all found headers   
   for ii = 1:length(header_start)
       frame.header_start = header_start(ii);
       % version(8bits)
       frame.version = videostream(frame.header_start + 4);  % 8bits
       % codec(8bits)
       frame.codec   = videostream(frame.header_start + 5);  % 8bits
       
       % header size(16bits) -  bytes
       ind1 = frame.header_start + 7;
       ind2 = frame.header_start + 6;
       
       frame.header_size = bin2dec([dec2bin(videostream(ind1),8) (dec2bin(videostream(ind2),8))]);
       
       % payload size(32bits)
       ind1 = frame.header_start + 11;
       ind2 = frame.header_start + 10;
       ind3 = frame.header_start + 9;
       ind4 = frame.header_start + 8;
       
       frame.payload = bin2dec([dec2bin(videostream(ind1),8) dec2bin(videostream(ind2),8) ...
           dec2bin(videostream(ind3),8) dec2bin(videostream(ind4),8)]);
       
       % encoded stream width(16bits)
       ind1 = frame.header_start + 13;
       ind2 = frame.header_start + 12;
       
       frame.width_encoded = bin2dec([dec2bin(videostream(ind1),8) dec2bin(videostream(ind2),8)]);
       
       % encoded stream height(16bits)
       ind1 = frame.header_start + 15;
       ind2 = frame.header_start + 14;
       
       frame.height_encoded = bin2dec([dec2bin(videostream(ind1),8) dec2bin(videostream(ind2),8)]);
       
       % display stream width(16bits)
       ind1 = frame.header_start + 17;
       ind2 = frame.header_start + 16;
       
       frame.width_display = bin2dec([dec2bin(videostream(ind1),8) dec2bin(videostream(ind2),8)]);
       
       % display stream height(16bits)
       ind1 = frame.header_start + 19;
       ind2 = frame.header_start + 18;
       
       frame.height_display = bin2dec([dec2bin(videostream(ind1),8) dec2bin(videostream(ind2),8)]);
       
       % frame number(32bits)
       ind1 = frame.header_start + 23;
       ind2 = frame.header_start + 22;
       ind3 = frame.header_start + 21;
       ind4 = frame.header_start + 20;
       
       frame.number = bin2dec([dec2bin(videostream(ind1),8) dec2bin(videostream(ind2),8) ...
           dec2bin(videostream(ind3),8) dec2bin(videostream(ind4),8)]);
    
       %% (to be decoded by soon to be Doctor Igor Pizetta  :) ) --------  
       % time stamp(32bits) - miliseconds 
       ind1 = frame.header_start + 27;
       ind2 = frame.header_start + 26;
       ind3 = frame.header_start + 25;
       ind4 = frame.header_start + 24;
       
       frame.time = bin2dec([dec2bin(videostream(ind1),8) dec2bin(videostream(ind2),8) ...
           dec2bin(videostream(ind3),8) dec2bin(videostream(ind4),8)]);
       
       %% ---------------------------------------------
       % frame type(8bits) - I=1|P=3 [I guess...]
       frame.type    = videostream(frame.header_start + 30);
       
       % control (8bits)
       frame.control = videostream(frame.header_start + 31);
       
       % byte position lower word (32bits)
       ind1 = frame.header_start + 35;
       ind2 = frame.header_start + 34;
       ind3 = frame.header_start + 33;
       ind4 = frame.header_start + 32;
       
       frame.payload_byte_position_lw = bin2dec([dec2bin(videostream(ind1),8) dec2bin(videostream(ind2),8) ...
           dec2bin(videostream(ind3),8) dec2bin(videostream(ind4),8)]);
       
       % byte position upper word (32bits)
       ind1 = frame.header_start + 39;
       ind2 = frame.header_start + 38;
       ind3 = frame.header_start + 37;
       ind4 = frame.header_start + 36;
       
       frame.payload_byte_position_uw = bin2dec([dec2bin(videostream(ind1),8) dec2bin(videostream(ind2),8) ...
           dec2bin(videostream(ind3),8) dec2bin(videostream(ind4),8)]);
       
       % ID(16bits)
       ind1 = frame.header_start + 41;
       ind2 = frame.header_start + 40;
       
       frame.id = bin2dec([dec2bin(videostream(ind1),8) dec2bin(videostream(ind2),8)]);
       
       % total slices
       frame.total_slices = videostream(frame.header_start + 42);
       % slice index
       frame.slice_index  = videostream(frame.header_start + 43);
       % Size of SPS inside payload (H264 only)
       frame.header1_size = videostream(frame.header_start + 44);
       % Size of PPS inside payload (H264 only)
       frame.header2_size = videostream(frame.header_start + 45);
       
       % advertised size
       ind1 = frame.header_start + 51;
       ind2 = frame.header_start + 50;
       ind3 = frame.header_start + 49;
       ind4 = frame.header_start + 48;
       
       frame.advertised_size = bin2dec([dec2bin(videostream(ind1),8) dec2bin(videostream(ind2),8) ...
           dec2bin(videostream(ind3),8) dec2bin(videostream(ind4),8)]);
       
       Frame{ii} = frame;
       
   end
end