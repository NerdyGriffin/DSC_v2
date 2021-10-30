%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  receiveControlParameters()
%Function receiveControlParameters()
%@end deftypefn

function ret = receiveControlParameters (dlg)
    updateProgressDlg(dlg, 'Receiving temperature control parameters from Arduino...');
    awaitResponse = true;

    while awaitResponse
        serialData = strip(readline(dlg.Arduino));

        if strlength(serialData) == 1

            switch strip(serialData)
                case 'c'
                    readline(dlg.Arduino);
                    serialData = strip(readline(dlg.Arduino));
                    disp(serialData)
                    [parsedData, dataIsNum] = str2num(serialData);

                    if dataIsNum && length(parsedData) == 4
                        dlg.StartTempCEditField.Value = parsedData(1); %double(readline(dlg.Arduino));
                        dlg.EndTempCEditField.Value = parsedData(2); %double(readline(dlg.Arduino));
                        dlg.RateCminEditField.Value = parsedData(3); %double(readline(dlg.Arduino));
                        dlg.HoldTimesecEditField.Value = parsedData(4); %double(readline(dlg.Arduino));
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
