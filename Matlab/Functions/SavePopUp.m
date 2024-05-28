% Save results popup
function choice = SavePopUp
    d = dialog('Position',[300 300 250 100], 'Name', 'Save');
    movegui(d, 'center');
    
    txt = uicontrol('Parent',d,...
           'Style','text',...
           'Position',[20 40 210 40],...
           'String','Save results (Log/Images)?');
       
    btn_n = uicontrol('Parent',d,...
           'Position',[40 20 70 25],...
           'String','No',...
           'Callback',@popup_callback_n);
       
    btn_y = uicontrol('Parent',d,...
           'Position',[140 20 70 25],...
           'String','Yes',...
           'Callback',@popup_callback_y);   
       
    choice = false;       
       
    % Wait for d to close before running to completion
    uiwait(d);
   
   function popup_callback_n(~, ~)
      delete(gcf);
      choice = false;
   end
   
   function popup_callback_y(~, ~)
      delete(gcf);
      choice = true;
   end
end