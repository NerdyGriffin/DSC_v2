%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  initializeSerialPort()
%Function initializeSerialPort()
%@end deftypefn

function ret = initializeSerialPort (dlg)
  % Get the list of available serial ports
  dlg.SerialPortList = serialportlist("available");
  disp('Available serial ports:')
  disp(dlg.SerialPortList)
  set(dlg.SerialPortComboBox, 'string', dlg.SerialPortList);
  set(dlg.StartExperimentButton, 'enable', 'off');

  if (isempty(dlg.SerialPortList))
    % Display a warning if no serial ports found
    message = 'No available serial ports were found. Make sure the arduino device is plugged into this computer via USB, and that it is not in use by another program (such as Arduino IDE).';
    errordlg(message, 'No Serial Device');
  else
    dlg.SerialPort = get(dlg.SerialPortComboBox, "string"){get (dlg.SerialPortComboBox, "value")};
    set(dlg.SerialPortComboBox, 'value', dlg.SerialPort);

    if isfield(dlg, 'Arduino') && exist(dlg.Arduino, "var") && isvalid(dlg.Arduino)
      delele(dlg.Arduino)
    end

    % Create an serial port object where you specify the USB port
    % (look in Arduino->tools -> port and the baud rate (9600)
    dlg.Arduino = serialport(dlg.SerialPort, 9600);

    % Create and display the progress bar
    dlg = updateProgressDlg(dlg, 0, 'Communicating with Arduino...');

    if isempty(fread (dlg.Arduino))
      message = sprintf("There was no response from the device on '%s'. Make sure that this is the correct serial port, and that the 'dsc\_arduino' sketch has been upload onto the Arduino.", dlg.SerialPort);
      errordlg(message, 'Failed to communicate with Arduino');
    else
      % Request the temperature control parameters from the arduino
      updateProgressDlg(dlg, 1/5, 'Communicating with Arduino...');
      flush(dlg.Arduino);
      updateProgressDlg(dlg, 2/5, 'Communicating with Arduino...');
      write(dlg.Arduino, 'i', 'char');
      updateProgressDlg(dlg, 3/5, 'Awaiting response from Arduino...');
      receivePIDGains(dlg);
      updateProgressDlg(dlg, 4/5, 'Awaiting response from Arduino...');
      receiveControlParameters(dlg);
      updateProgressDlg(dlg, 5/5, 'Awaiting response from Arduino...');
      set(dlg.StartExperimentButton, 'enable', 'on');
    end

    % Close the progress bar
    dlg = closeProgressDlg(dlg);

  end

  ret = 0;
endfunction
