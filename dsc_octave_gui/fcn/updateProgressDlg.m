%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  updateProgressDlg()
%Function updateProgressDlg()
%@end deftypefn

function ret = updateProgressDlg (dlg, fraction, message)

  if isfield(dlg, 'SharedProgressDlg') && isvalid(dlg.SharedProgressDlg)
    waitbar(fraction, dlg.SharedProgressDlg, sprintf('%s %.2f%%', message, 100 * i));
  else
    % Create and display the progress bar
    h = waitbar(fraction, sprintf('%s %.2f%%', message, 100 * i));
    %dlg.SharedProgressDlg = h;
    setfield(dlg, 'SharedProgressDlg', h);
  end

  drawnow

  ret = dlg;
endfunction
