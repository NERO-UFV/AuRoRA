%% Frame I type finder
% This function look for all I-type frames in a cell.
% Each cell element contains a frame header structure
%
% Macroblock types (sdk2.0 pag.56)
% I frame intra 16x16 = 1 : complete frame
% I frame intra 4x4   = 2(ou 0??) : complete frame
% P frame = 3 : holds only changes in the image from previous 
% frame; needs previous frame to be decoded
%
% Input: 
%     Frame_headers = cell array with all frame headers struct
%     videostream   = data received vector
% Output:
%     indexI        = all Frame I-type index
%     frameI        = all complete I-frames  


function [indexI,frameI] = Iframe_finder(Frame_headers,videostream)
global Q
    n = 1;
    x = 1;
    % Find all I-frames
    for ii = 1:length(Frame_headers)

        a = Frame_headers{ii};

        if a.type == 1
%             disp('I-frame');
            indexI(n) = ii;
            n = n+1;

        elseif a.type == 3
%             disp('P-frame');
        else
            disp('Unknown frame type!');
            indexX(x) = ii;
            x = x+1;

        end
    end
  
    
% Get macroblocks from I frames
% # MACROBLOCK (Group of blocks subdivision)
%     --------------...--
%    | 0 | 1 | 2 |  ...  |   <-- slice of the original image  
%     --------------...-- 
%
%       ---------      blue-dif.   red-difference
%      | Y0 | Y1 |       ----          ----
%      |----+----|      | Cb |        | Cr |
%      | Y2 | Y3 |       ----          ----
%       ---------         Chroma components
%     Luma component
%      (greyscale)
%
%  
% Y0,Y1,Y2,Y3,Cb,Cr = 8x8 blocks
%
%  Stream format:
% 
%   ____ ____ ____ ____ ____ ____
%  | Y0 | Y1 | Y2 | Y3 | Cb | Cr | ...
%  |____|____|____|____|____|____|
% 


% Blocks Parameters
mb_height = 16;      % macroblock heigth
mb_width  = 16;      % macroblock width
submb     = 8;    % submacroblock size
block     = submb^2; % number of pixels from each macroblock square subdivision

for k = 1:length(indexI)
    
    % get 2 consecutive frames to determine picture data 
    f1 = Frame_headers{indexI(k)};
    f2 = Frame_headers{indexI(k)+1};
   
    % determine picture data (index) range
    ind0  = f1.header_start + f1.header_size; % dúvida, 64 ou 68bits?(sdk p.60)
%     ind1  = f2.header_start - 1;
    
%     pixels = ind1 - ind0; % total pixels per frame
    start = ind0;         % initial frame index
    
    % number of macroblocks considering picture width and heigth
    mb_numberw = floor(f1.width_encoded/mb_width);   % width
    mb_numberh = floor(f1.height_encoded/mb_height); % heigth
    
    try
    % Generates all macroblocks 
    for n1 = 1:mb_numberh
        for n2 = 1:mb_numberw
            
 % subblock 8x8 ////////////////////////////////////////////////////////

            if submb==8
            % block index (auxiliary only)
            i(1) = start + block -1;
            for m=2:5
                i(m) = i(m-1) + block;
            end
            
            % block vectors
            y{1} = videostream(start : i(1));  
            for m=2:4
                y{m} = videostream(i(m-1) + 1 : i(m));
            end
                            
%             cb = videostream(i(4) + 1 : i(5));
%             cr = videostream(i(5) + 1 : i(5) + block);
%             
            % zig zag inverse ordering for submacroblocks (8x8) 
            for m=1:4
                Y{m} = invzigzag(y{m},submb,submb);
            end

%             CB = invzigzag(cb,submb,submb);
%             CR = invzigzag(cr,submb,submb);
%            
%             Y0 = reshape(y0,submb,submb);
%             Y1 = reshape(y1,submb,submb);
%             Y2 = reshape(y2,submb,submb);
%             Y3 = reshape(y3,submb,submb);
%             CB = reshape(cb,submb,submb);
%             CR = reshape(cr,submb,submb);

%             % Quantization 
%             Q = quant(submb,submb,1);
%            for m = 1:4 
%               Y{m} = Y{m}.*Q; %-1/2
%            end
                                  
            % Inverse Cosine discrete transform 
            for m=1:4
            Y{m} = round(idct2(Y{m},submb,submb));
            end
      
            % Macroblock 
%             MB{n1,n2} = [Y0 Y1; Y2 Y3];   % reading only luma
            MB{n1,n2} = [Y{1} Y{2}; Y{3} Y{4}];   % reading only luma

            start = start + 6*block;      % establish new macroblock start index   
%             color1{n1} = CB;
%             color2{n1} = CR;

   % subblock 4x4 /////////////////////////////////////////////////////
            elseif submb==4
         i(1) = start + block -1;
            for m=2:19
                i(m) = i(m-1) + block;
            end
            
            % block vectors
            y{1} = videostream(start : i(1));  
            for m=2:16
                y{m} = videostream(i(m-1) + 1 : i(m));
            end
                                
%             cb = videostream(i(4) + 1 : i(5));
%             cr = videostream(i(5) + 1 : i(5) + block);
            
            % zig zag inverse ordering for submacroblocks (8x8) 
            for m=1:16
                Y{m} = invzigzag(y{m},submb,submb);
            end

%             CB = invzigzag(cb,submb,submb);
%             CR = invzigzag(cr,submb,submb);

%             % Quantization 
             Q = quant(submb,submb,1);
            
           for m = 1:16 
              Y{m} = Y{m}.*Q; %-1/2
           end   

            % Inverse Cosine discrete transform 
            for m=1:16
            Y{m} = round(idct2(Y{m},submb,submb));
            end
     
            % Macroblock 
%             MB{n1,n2} = [Y0 Y1; Y2 Y3];   % reading only luma
            MB{n1,n2} = [Y{1}  Y{2}  Y{5}  Y{6}; 
                         Y{3}  Y{4}  Y{7}  Y{8};
                         Y{9}  Y{10} Y{13} Y{14}; 
                         Y{11} Y{12} Y{15} Y{16}];   % reading only luma

            start = start + 20*block;      % establish new macroblock start index   
%             color1{n1} = CB;
%             color2{n1} = CR;
            end 
            
        end
    end       
    
   % Mount Macroblocks
   % generates horizontal slices (GOB)
   for kh = 1:mb_numberh
       
       slice(kh).hor = [];
        
       for kw = 1:mb_numberw
            slice(kh).hor = [slice(kh).hor MB{kh,kw}];          
       end
       
   end
   
   % Group all GOBs into a complete picture
   picture = [];
   
   for ii = 1:mb_numberh
       
      picture = [picture; slice(ii).hor];   
      
   end
   
   % save each picture in a cell element
   frameI{k} = picture;
   
    catch
       disp(['Not possible to mount I-frame ',num2str(k)]); 
       disp('(Not enough data to mount all macroblocks - likely)');
       
    end
   
end     
    
end
