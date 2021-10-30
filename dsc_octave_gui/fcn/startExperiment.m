%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  startExperiment()
%Function startExperiment()
%@end deftypefn

function ret = startExperiment (dlg)
  flush(dlg.Arduino);
  write(dlg.Arduino, 's', 'char');

  awaitStart = true;

  while awaitStart
    serialData = strip(readline(dlg.Arduino));

    if strlength(serialData) == 1

      switch strip(serialData)
        case 's'
          setRunningUI(dlg);
          receiveSerialData(dlg);
          awaitStart = false;
        case 'x'
          setIdleUI(dlg);
          disp('Received end signal')
          awaitStart = false;
        otherwise
          disp('Unrecognized data flag while awaiting start response:')
          disp(serialData);
      end

    end

  end

  ret = 0;
endfunction
