

ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'land', ...
                         'Callback', 'a=3', ...
                         'Position', [50 50 400 300]);

                     
                     %%
 try    
    while toc(t) < tmax
        
       if toc(tc) > To
            tc = tic;
            
            if ~ishandle(ButtonHandle)
                disp('Bebop Landing');
                break;
            end
        
        
       end
               
    end
   
 catch Exception    
     % B.rLand;          
      disp("Pouso forcado via comando da estrutura Try Catch");
      disp(toStringJSON(Exception));
end