function [] = rPlotPerformanceAnalysisGeneral(IAE_ITAE_IASC, tmax)
%****** Função para plotar os principais gráficos ******%

fInd0 = figure('Name','Análise de Performance','NumberTitle','off');
fInd0.Position = [437 44 700 493]; 
    
    figure(fInd0);
    plot(IAE_ITAE_IASC(:,4)*30e-3,IAE_ITAE_IASC(:,1), 'LineWidth', 1)
    grid on
    axis([0 tmax round(min(IAE_ITAE_IASC(:,1))-5) round(max(IAE_ITAE_IASC(:,1))+5)])
    xlabel('Atraso de Tempo [$s$]','FontSize',14,'FontWeight','bold','interpreter','Latex')
    ylabel('IAE','FontSize',14,'FontWeight','bold','interpreter','Latex')

name_fig = sprintf('C:%cUsers%cleo_j%cDropbox%cDropbox%cAuRoRA 2018%cTime-delayed Communication%cFigs%cfigPos IAEIndices_Desempenho_Geral', '\', '\', '\', '\', '\', '\', '\', '\');
saveFigure(fInd0, name_fig);     % Salva a figura .fig
saveas(fInd0, name_fig, 'epsc'); % Salva a figura .eps
saveas(fInd0, name_fig, 'pdf');  % Salva a figura .pdf
saveas(fInd0, name_fig, 'png');  % Salva a figura .png
    

fInd1 = figure('Name','Análise de Performance','NumberTitle','off');
fInd1.Position = [437 44 700 493];
 
    figure(fInd1);
    
    plot(IAE_ITAE_IASC(:,4)*30e-3,IAE_ITAE_IASC(:,2), 'LineWidth', 1)
    grid on
    axis([0 tmax round(min(IAE_ITAE_IASC(:,2))-5) round(max(IAE_ITAE_IASC(:,2))+5)])
    xlabel('Atraso de Tempo [$s$]','FontSize',14,'FontWeight','bold','interpreter','Latex')
    ylabel('ITAE','FontSize',14,'FontWeight','bold','interpreter','Latex')

name_fig = sprintf('C:%cUsers%cleo_j%cDropbox%cDropbox%cAuRoRA 2018%cTime-delayed Communication%cFigs%cfigPos ITAEIndices_Desempenho_Geral', '\', '\', '\', '\', '\', '\', '\', '\');
saveFigure(fInd1, name_fig);     % Salva a figura .fig
saveas(fInd1, name_fig, 'epsc'); % Salva a figura .eps
saveas(fInd1, name_fig, 'pdf');  % Salva a figura .pdf
saveas(fInd1, name_fig, 'png');  % Salva a figura .png


fInd2 = figure('Name','Análise de Performance','NumberTitle','off');
fInd2.Position = [437 44 700 493];

    figure(fInd2);
    plot(IAE_ITAE_IASC(:,4)*30e-3,IAE_ITAE_IASC(:,3), 'LineWidth', 1)
    grid on
    axis([0 tmax round(min(IAE_ITAE_IASC(:,3))-5) round(max(IAE_ITAE_IASC(:,3))+5)])
    xlabel('Atraso de Tempo [$s$]','FontSize',14,'FontWeight','bold','interpreter','Latex')
    ylabel('IASC','FontSize',14,'FontWeight','bold','interpreter','Latex')


name_fig = sprintf('C:%cUsers%cleo_j%cDropbox%cDropbox%cAuRoRA 2018%cTime-delayed Communication%cFigs%cfigPos IASCIndices_Desempenho_Geral', '\', '\', '\', '\', '\', '\', '\', '\');
saveFigure(fInd2, name_fig);     % Salva a figura .fig
saveas(fInd2, name_fig, 'epsc'); % Salva a figura .eps
saveas(fInd2, name_fig, 'pdf');  % Salva a figura .pdf
saveas(fInd2, name_fig, 'png');  % Salva a figura .png
%-------------------------------------------------------------------------

end