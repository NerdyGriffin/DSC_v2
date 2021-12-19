%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  refreshLivePlot()
%Function refreshLivePlot()
%@end deftypefn

function ret = refreshLivePlot (dlg, elapsedTimeArray, ...
    targetTempArray, refTempArray, sampTempArray)
  %plot (dlg.Main_Plot, x, y, "k:"); %, x, y, "b", x, y, "r");
  plot (dlg.Main_Plot, elapsedTimeArray, targetTempArray, "k:",
  elapsedTimeArray, refTempArray, "b",
  elapsedTimeArray, sampTempArray, "r");
  legend(dlg.Main_Plot, 'Target Temperature', 'Reference Sample', 'Test Sample');

  ret = dlg;
endfunction
