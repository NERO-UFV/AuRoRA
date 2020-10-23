function clickFunction(src,~)
    pt = get(gca,'CurrentPoint');
    assignin('base','clickPoint',pt)
end

