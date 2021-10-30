%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  receiveSerialData()
%Function receiveSerialData()
%@end deftypefn

function ret = receiveSerialData (dlg)
  updateProgressDlg(dlg, 'Awaiting initial data...');

  dlg.Data.startTemp = dlg.StartTempCEditField.Value;
  dlg.Data.endTemp = dlg.EndTempCEditField.Value;
  dlg.Data.rampUpRate = dlg.RateCminEditField.Value;
  dlg.Data.holdTime = dlg.HoldTimesecEditField.Value;

  dlg.Data.startDateTime = datetime;

  if not(isfolder('autosave'))
    mkdir('autosave');
  end

  dlg.Data.elapsedTime = zeros(1, dlg.PlotRefreshDelay);
  dlg.Data.targetTemp = zeros(1, dlg.PlotRefreshDelay);
  dlg.Data.refTemp = zeros(1, dlg.PlotRefreshDelay);
  dlg.Data.sampTemp = zeros(1, dlg.PlotRefreshDelay);
  dlg.Data.refCurrent = zeros(1, dlg.PlotRefreshDelay);
  dlg.Data.sampCurrent = zeros(1, dlg.PlotRefreshDelay);
  dlg.Data.refHeatFlow = zeros(1, dlg.PlotRefreshDelay);
  dlg.Data.sampHeatFlow = zeros(1, dlg.PlotRefreshDelay);
  dlg.Data.refDutyCycle = zeros(1, dlg.PlotRefreshDelay);
  dlg.Data.sampDutyCycle = zeros(1, dlg.PlotRefreshDelay);

  dataLength = 0;

  experimentIsRunning = true;

  while experimentIsRunning
    serialData = strip(readline(dlg.Arduino));

    if strlength(serialData) == 1

      switch strip(serialData)
        case 'x'
          experimentIsRunning = false;
          disp('Received end signal')
          updateProgressDlg(dlg, 'Awaiting response from Arduino...');
          receivePIDGains(dlg);
          receiveControlParameters(dlg);
        otherwise
          disp('Unrecognized data flag while awaiting data:')
          disp(serialData)
      end

    else
      [parsedData, dataIsNum] = str2num(serialData);

      if dataIsNum && length(parsedData) == 10
        dataLength = dataLength + 1;
        dlg.Data.elapsedTime(dataLength) = parsedData(1); %str2double(parsedData{1});
        dlg.Data.targetTemp(dataLength) = parsedData(2); %str2double(parsedData{2});
        dlg.Data.refTemp(dataLength) = parsedData(3); %str2double(parsedData{3});
        dlg.Data.sampTemp(dataLength) = parsedData(4); %str2double(parsedData{4});
        dlg.Data.refCurrent(dataLength) = parsedData(5); %str2double(parsedData{5});
        dlg.Data.sampCurrent(dataLength) = parsedData(6); %str2double(parsedData{6});
        dlg.Data.refHeatFlow(dataLength) = parsedData(7); %str2double(parsedData{7});
        dlg.Data.sampHeatFlow(dataLength) = parsedData(8); %str2double(parsedData{8});
        dlg.Data.refDutyCycle(dataLength) = parsedData(9); %str2double(parsedData{9});
        dlg.Data.sampDutyCycle(dataLength) = parsedData(10); %str2double(parsedData{10});

        if ~mod(dataLength, dlg.DataRefreshDelay)
          updateLiveData(dlg, ...
            dlg.Data.elapsedTime(dataLength), ...
            dlg.Data.targetTemp(dataLength), ...
            dlg.Data.refTemp(dataLength), ...
            dlg.Data.sampTemp(dataLength), ...
            dlg.Data.refCurrent(dataLength), ...
            dlg.Data.sampCurrent(dataLength), ...
            dlg.Data.refDutyCycle(dataLength), ...
            dlg.Data.sampDutyCycle(dataLength));
        end

        if ~mod(dataLength, dlg.PlotRefreshDelay)
          refreshLivePlot(dlg, ...
            dlg.Data.elapsedTime, dlg.Data.targetTemp, ...
            dlg.Data.refTemp, dlg.Data.sampTemp);
          % Close the progress bar
          if isvalid(dlg.SharedProgressDlg)
            close(dlg.SharedProgressDlg)
          end

        end

      else
        disp(parsedData)
      end

    end

  end

  dlg.Data.dataLength = dataLength;

  % Close the progress bar
  if isvalid(dlg.SharedProgressDlg)
    close(dlg.SharedProgressDlg)
  end

  saveData(dlg, dlg.Data);

  updateLiveData(dlg, ...
    dlg.Data.elapsedTime(dataLength), ...
    dlg.Data.targetTemp(dataLength), ...
    dlg.Data.refTemp(dataLength), ...
    dlg.Data.sampTemp(dataLength), ...
    dlg.Data.refCurrent(dataLength), ...
    dlg.Data.sampCurrent(dataLength), ...
    dlg.Data.refDutyCycle(dataLength), ...
    dlg.Data.sampDutyCycle(dataLength));

  refreshLivePlot(dlg, dlg.Data.elapsedTime, dlg.Data.targetTemp, ...
    dlg.Data.refTemp, dlg.Data.sampTemp);

  setIdleUI(dlg);

  ret = 0;
endfunction
