%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  refreshLivePlot()
%Function refreshLivePlot()
%@end deftypefn

function ret = refreshLivePlot (dlg, init = false)

    %
    %  Add yout code here
    %

    if (init)
        x = [0, 1];
        y = [0, 0];
    else
        plot (dlg.Main_Plot, Data.elapsedTimeArray, Data.targetTempArray, "k:",
        Data.elapsedTimeArray, Data.refTempArray, "b",
        Data.elapsedTimeArray, Data.sampTempArray, "r");
    endif

    plot (dlg.Main_Plot, x, y, "k:"); %, x, y, "b", x, y, "r");

    ret = 0;
endfunction
