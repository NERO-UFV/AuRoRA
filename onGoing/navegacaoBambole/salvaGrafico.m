function salvaGrafico(nome)
%salvaGrafico Salva figura atual em pdf
%   Tomando nome como string de entrada, salva .pdf
%   vetorizando a figura aberta por último
    
    try 
    	cd imagens
    catch
        mkdir imagens
        cd imagens
    end 

    set(gcf,'Units','inches');
    screenposition = get(gcf,'Position');
    set(gcf,...
        'PaperPosition',[0 0 screenposition(3:4)],...
        'PaperSize',[screenposition(3:4)]);
    print(gcf,'-dpdf',nome)
    print(gcf,'-dsvg',nome)
    
    cd ..
end

