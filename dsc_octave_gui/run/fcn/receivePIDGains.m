%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  receivePIDGains()
%Function receivePIDGains()
%@end deftypefn

function ret = receivePIDGains (dlg)
  % Receive the PID gain constants via the serial bus
  updateProgressDlg(dlg, 'Receiving PID Gains from Arduino...');
  awaitResponse = true;

  while awaitResponse
    serialData = strip(readline(dlg.Arduino));

    if strlength(serialData) == 1

      switch strip(serialData)
        case 'k'
          readline(dlg.Arduino);
          serialData = strip(readline(dlg.Arduino));
          [parsedData, dataIsNum] = str2num(serialData);

          if dataIsNum && length(parsedData) == 3
            dlg.Data.Kp = parsedData(1); %double(readline(dlg.Arduino));
            dlg.Data.Ki = parsedData(2); %double(readline(dlg.Arduino));
            dlg.Data.Kd = parsedData(3); %double(readline(dlg.Arduino));
          end

          awaitResponse = false;
        case 'x'
          setIdleUI(dlg);
          disp('Received end signal')
          awaitResponse = false;
        otherwise
          disp('Unrecognized data flag while awaiting PID Gains:')
          disp(serialData);
      end

    end

  end

  ret = 0;
endfunction
