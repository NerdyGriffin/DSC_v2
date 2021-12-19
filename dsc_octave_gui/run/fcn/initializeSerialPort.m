%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  initializeSerialPort()
%Function initializeSerialPort()
%@end deftypefn

function ret = initializeSerialPort (dlg)
  % Get the list of available serial ports
  dlg.SerialPortList = serialportlist("available");
  disp('Available serial ports:')
  disp(dlg.SerialPortList)
  set(dlg.StartExperimentButton, 'enable', 'off');

  if (isempty(dlg.SerialPortList))
    % Display a warning if no serial ports found
    message = 'No available serial ports were found. Make sure the arduino device is plugged into this computer via USB, and that it is not in use by another program (such as Arduino IDE).';
    errordlg(message, 'No Serial Device');
  else
    prevSerialPortList = get(dlg.SerialPortComboBox, 'string');
    set(dlg.SerialPortComboBox, 'string', dlg.SerialPortList);

    if (isempty(prevSerialPortList))
      % Set the default serial port to the last port in the list
      set(dlg.SerialPortComboBox, 'value', length(get(dlg.SerialPortComboBox, 'string')));
    end

    dlg.SerialPort = get(dlg.SerialPortComboBox, 'string'){get (dlg.SerialPortComboBox, 'value')};

    if isfield(dlg, 'Arduino') && exist(dlg.Arduino, "var")
      delele(dlg.Arduino)
    end

    % Create an serial port object where you specify the USB port
    % (look in Arduino->tools -> port and the baud rate (9600)
    dlg.Arduino = serialport(dlg.SerialPort, 9600);
    pause(1); % Optional wait for device to wake up

    % Create and display the progress bar
    dlg.SharedProgressDlg = updateProgressDlg(dlg, 0, 'Communicating with Arduino...');

    if isempty(readline(dlg.Arduino))
      message = sprintf("There was no response from the device on '%s'. Make sure that this is the correct serial port, and that the 'dsc\_arduino' sketch has been upload onto the Arduino.", dlg.SerialPort);
      errordlg(message, 'Failed to communicate with Arduino');
    else
      % Request the temperature control parameters from the arduino
      flush(dlg.Arduino);
      dlg.SharedProgressDlg = updateProgressDlg(dlg, 1/5, 'Communicating with Arduino...');

      write(dlg.Arduino, 'i', 'char');
      dlg.SharedProgressDlg = updateProgressDlg(dlg, 2/5, 'Communicating with Arduino...');

      receivePIDGains(dlg, 3/5);

      receiveControlParameters(dlg, 4/5);

      set(dlg.StartExperimentButton, 'enable', 'on');
      dlg.SharedProgressDlg = updateProgressDlg(dlg, 5/5, 'Finished communicating with Arduino.');
    end

    % Close the progress bar
    dlg = closeProgressDlg(dlg);

  end

  ret = dlg;
endfunction
