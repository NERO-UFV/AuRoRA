function [ListaAbertaAtualizada,ListaFechadaAtualizada] = ExpandeNodo(occ_grid,ListaAberta,ListaFechada,pos,tamX,tamY,fimX,fimY)
paiX = ceil(pos/tamY);
if pos <= tamY
    paiY = pos;
else
    paiY = rem(pos,tamY);
    if paiY == 0
        paiY = tamY;
    end
end
for i= 1:-1:-1
    for j= 1:-1:-1
        if (i~=0 || j~=0)
            expandeX = paiX + i;
            expandeY = paiY + j;
            if expandeX>0 && expandeY>0 && expandeX <= tamX && expandeY <= tamY
                custoAtual = ListaFechada(pos,6);                
                custoNodo = custoAtual + norm(grid2world(occ_grid,[paiX paiY])-grid2world(occ_grid,[expandeX expandeY]));
                posNodo = (expandeX-1)*tamY+expandeY;
                hn = norm(grid2world(occ_grid,[expandeX expandeY])-grid2world(occ_grid,[fimX fimY]));
                %hn = abs(expandeX - fimX)+abs(expandeY - fimY);
                if ListaAberta(posNodo,1) == true && ListaAberta(posNodo,6) > custoNodo
                    ListaAberta(posNodo,1) = false;
                end
                if ListaFechada(posNodo,1) == true && ListaFechada(posNodo,6) > custoNodo
                    ListaFechada(posNodo,1) = false;
                end
                if (ListaAberta(posNodo,1) == false) && (ListaFechada(posNodo,1) == false)
                    ListaAberta(posNodo,:) = [true expandeX expandeY paiX paiY custoNodo hn custoNodo+hn];
                end
            end
        end
    end
end
ListaAbertaAtualizada = ListaAberta;
ListaFechadaAtualizada = ListaFechada;
end