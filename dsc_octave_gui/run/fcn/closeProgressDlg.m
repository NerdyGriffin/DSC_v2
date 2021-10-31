%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  closeProgressDlg()
%Function closeProgressDlg()
%@end deftypefn

function ret =  closeProgressDlg (dlg)
  % Close the progress bar
  if isfield(dlg, 'SharedProgressDlg') && isvalid(dlg.SharedProgressDlg)
    close(dlg.SharedProgressDlg)
  end

  ret = 0;
endfunction
