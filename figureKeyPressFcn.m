
function figureKeyPressFcn(hObject, eventdata, handles)
global w;
global v;
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see FIGURE)
%   Key: name of the key that was pressed, in lower case
%   Character: character interpretation of the key(s) that was pressed
%   Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

% add this part as an experiment and see what happens!
%eventdata % Let's see the KeyPress event data
disp(eventdata.Key) % Let's display the key, for fun!

switch(eventdata.Key)
    case 'w'
        v = 0.2;
    case 's'
        v = -0.2;
    case 'a'
        w = 0.2;
    case 'd'
        w = -0.2;
    case 'q'
        v = 0;
    case 'e'
        w = 0;
    otherwise
end
end