%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  closeProgressDlg()
%Function closeProgressDlg()
%@end deftypefn

function ret = closeProgressDlg (dlg)
  % Close the progress bar
  if isfield(dlg, 'SharedProgressDlg')
    close(dlg.SharedProgressDlg)
  end

  ret = dlg;
endfunction
