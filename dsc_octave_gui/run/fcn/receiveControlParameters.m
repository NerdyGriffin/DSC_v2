%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  receiveControlParameters()
%Function receiveControlParameters()
%@end deftypefn

function ret = receiveControlParameters (dlg, fraction = 0)
  dlg.SharedProgressDlg = updateProgressDlg(dlg, fraction, 'Receiving temperature control parameters from Arduino...');
  awaitResponse = true;

  while awaitResponse
    serialData = strtrim(readline(dlg.Arduino));

    if length(serialData) == 1

      switch strtrim(serialData)
        case 'c'
          readline(dlg.Arduino);
          serialData = strtrim(readline(dlg.Arduino));
          disp(serialData)
          [parsedData, dataIsNum] = str2num(serialData);

          if dataIsNum && length(parsedData) == 4
            set(dlg.StartTempEditField, 'value', parsedData(1)); %double(readline(dlg.Arduino));
            set(dlg.EndTempEditField, 'value', parsedData(2)); %double(readline(dlg.Arduino));
            set(dlg.RateEditField, 'value', parsedData(3)); %double(readline(dlg.Arduino));
            set(dlg.HoldTimeEditField, 'value', parsedData(4)); %double(readline(dlg.Arduino));
          end

          awaitResponse = false;
        case 'x'
          setIdleUI(dlg);
          disp('Received end signal')
          awaitResponse = false;
        otherwise
          disp('Unrecognized data flag while awaiting control params:')
          disp(serialData);
      end

    end

  end

  ret = 0;
endfunction
