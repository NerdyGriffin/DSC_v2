%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  updateProgressDlg()
%Function updateProgressDlg()
%@end deftypefn

function ret = updateProgressDlg (dlg, fraction, message)

  if isfield(dlg, 'SharedProgressDlg') && isvalid(dlg.SharedProgressDlg)
    waitbar(fraction, dlg.SharedProgressDlg, sprintf('%s %.2f%%', message, 100 * i));
  else
    % Create and display the progress bar
    dlg.SharedProgressDlg = waitbar(fraction, sprintf('%s %.2f%%', message, 100 * i));
  end

  drawnow

  ret = 0;
endfunction
