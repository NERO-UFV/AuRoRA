function OnMouse(hObject,~)
axes_handle = gca;
pt = get(axes_handle, 'CurrentPoint');
clicked_x=pt(1,1)
clicked_y=pt(1,2)