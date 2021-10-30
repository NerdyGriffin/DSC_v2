%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  initializeSerialPort()
%Function initializeSerialPort()
%@end deftypefn

function ret = initializeSerialPort (dlg)
    % Get the list of available serial ports
    dlg.SerialPortList = serialportlist("available");
    disp('Available serial ports:')
    disp(dlg.SerialPortList)
    dlg.StartExperimentButton.Enable = 'off';

    if (isempty(dlg.SerialPortList))
        % Display a warning if no serial ports found
        message = 'No available serial ports were found. Make sure the arduino device is plugged into this computer via USB, and that it is not in use by another program (such as Arduino IDE).';
        uialert(dlg.UIFigure, message, 'No Serial Device');
    else

        if (isempty(dlg.SerialPortEditField.Value))
            % Set the default serial port to the last port in the list
            dlg.SerialPortEditField.Value = dlg.SerialPortList(end);
        end

        if ~any(contains(dlg.SerialPortList, dlg.SerialPortEditField.Value))
            message = sprintf("%s is not in the list of available serial ports", dlg.SerialPortEditField.Value);
            uialert(dlg.UIFigure, message, 'Invalid Serial Port')
        else
            dlg.SerialPort = dlg.SerialPortEditField.Value;

            if exist(dlg.Arduino, "var") && isvalid(dlg.Arduino)
                delele(dlg.Arduino)
            end

            % Create an serial port object where you specify the USB port
            % (look in Arduino->tools -> port and the baud rate (9600)
            dlg.Arduino = serialport(dlg.SerialPort, 9600);

            % Create and display the progress bar
            updateProgressDlg(dlg, 'Awaiting response from Arduino...');

            if isempty(readline(dlg.Arduino))
                message = sprintf("There was no response from the device on ' %s'. Make sure that this is the correct serial port, and that the 'dsc_arduino' sketch has been upload onto the Arduino.", dlg.SerialPort);
                uialert(dlg.UIFigure, message, 'Failed to communicate with Arduino');
            else
                % Request the temperature control parameters from the arduino
                flush(dlg.Arduino);
                write(dlg.Arduino, 'i', 'char');
                receivePIDGains(dlg);
                receiveControlParameters(dlg);
                dlg.StartExperimentButton.Enable = 'on';
            end

            % Close the progress bar
            if isvalid(dlg.SharedProgressDlg)
                close(dlg.SharedProgressDlg)
            end

        end

    end

    ret = 0;
endfunction
