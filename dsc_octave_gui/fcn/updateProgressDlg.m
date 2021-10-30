%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  updateProgressDlg()
%Function updateProgressDlg()
%@end deftypefn

function ret = updateProgressDlg (dlg, message)

    %
    %  Add yout code here
    %

    if isvalid(dlg.SharedProgressDlg)
        dlg.SharedProgressDlg.Message = message;
    else
        % Create and display the progress bar
        dlg.SharedProgressDlg = uiprogressdlg(dlg.UIFigure, 'Title', 'Communicating with Arduino', ...
            'Message', message, 'Indeterminate', 'on');
    end

    drawnow

    ret = 0;
endfunction
