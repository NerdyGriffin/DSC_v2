%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  saveAxesAsFigure()
%Function saveAxesAsFigure()
%@end deftypefn

function ret = saveAxesAsFigure (dlg, figureFilename)
  fignew = figure('Visible', 'off'); % Invisible figure
  newAxes = copyobj(dlg.UIAxes, fignew); % Copy the appropriate axes
  set(newAxes, 'Position', get(groot, 'DefaultAxesPosition')); % The original position is copied too, so adjust it.
  set(fignew, 'CreateFcn', 'set(gcbf,''Visible'',''on'')'); % Make it visible upon loading
  savefig(fignew, figureFilename);
  delete(fignew);

  ret = 0;
endfunction
