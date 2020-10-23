function clickFunctionfloor(src,~)
    pt = get(gca,'CurrentPoint');
    pt = floor(pt);
    assignin('base','clickPoint',pt)
end

