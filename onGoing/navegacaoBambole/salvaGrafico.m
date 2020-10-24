function salvaGrafico(nome)
%salvaGrafico Salva figura atual em pdf
%   Tomando nome como string de entrada, salva .pdf
%   vetorizando a figura aberta por último
    
    set(gcf,'Units','inches');
    screenposition = get(gcf,'Position');
    set(gcf,...
        'PaperPosition',[0 0 screenposition(3:4)],...
        'PaperSize',[screenposition(3:4)]);
    print(gcf,'-dpdf',nome)
end

