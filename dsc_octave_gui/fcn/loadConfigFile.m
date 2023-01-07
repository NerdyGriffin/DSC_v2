%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  loadConfigFile()
%Function loadConfigFile()
%@end deftypefn

function configLoadStatus = loadConfigFile (dlg)
  %   Load control parameters from a .ini file

  ini = IniConfig();

  % Prompt the user to select a file
  [configFileName, configFilePath] = uigetfile('*.ini');

  % Re-focus the dlg window
  drawnow;
  figure(dlg.UIFigure)

  switch configFileName
    case 0
      % Cancel the read operation and return an empty array
      % if the user closes the file selection window
      configLoadStatus = false;
      return

    otherwise
      loadingMessage = 'Loading config file...';

      % Create and display the progress bar
      dlg.SharedProgressDlg = updateProgressDlg(dlg, 0, loadingMessage);

      % Create fully-formed filename as a string
      configFullPath = fullfile(configFilePath, configFileName);
      dlg.SharedProgressDlg = updateProgressDlg(dlg, 1/4, loadingMessage);

      % Read the .ini file
      ini.ReadFile(configFullPath);
      dlg.SharedProgressDlg = updateProgressDlg(dlg, 2/4, loadingMessage);

      PIDSection = 'PID Settings';

      if ini.IsSections(PIDSection)

        if ini.IsKeys(PIDSection, 'Kp')
          dlg.Data.Kp = ...
            ini.GetValues('PID Settings', 'Kp');
        end

        if ini.IsKeys(PIDSection, 'Ki')
          dlg.Data.Ki = ...
            ini.GetValues('PID Settings', 'Ki');
        end

        if ini.IsKeys(PIDSection, 'Kd')
          dlg.Data.Kd = ...
            ini.GetValues('PID Settings', 'Kd');
        end

      else
        % Close the progress bar
        dlg = closeProgressDlg(dlg);

        warningMessage = sprintf("The selected .ini file does not contain a [%s] section", PIDSection);
        errordlg(warningMessage, 'Invalid File');
        configLoadStatus = false;
        return
      end

      dlg.SharedProgressDlg = updateProgressDlg(dlg, 3/4, loadingMessage);

      TempControlSection = 'Temperature Control';

      if ini.IsSections(TempControlSection)

        if ini.IsKeys(TempControlSection, 'startTemp')
          dlg.Data.startTemp = ...
            ini.GetValues(TempControlSection, 'startTemp');
          set(dlg.StartTempEditField, 'value', dlg.Data.startTemp);
        end

        if ini.IsKeys(TempControlSection, 'endTemp')
          dlg.Data.endTemp = ...
            ini.GetValues(TempControlSection, 'endTemp');
          set(dlg.EndTempEditField, 'value', dlg.Data.endTemp);
        end

        if ini.IsKeys(TempControlSection, 'rampUpRate')
          dlg.Data.rampUpRate = ...
            ini.GetValues(TempControlSection, 'rampUpRate');
          set(dlg.RateEditField, 'value', dlg.Data.rampUpRate);
        end

        if ini.IsKeys(TempControlSection, 'holdTime')
          dlg.Data.holdTime = ...
            ini.GetValues(TempControlSection, 'holdTime');
          set(dlg.HoldTimeEditField, 'value', dlg.Data.holdTime);
        end

      else
        % Close the progress bar
        dlg = closeProgressDlg(dlg);

        warningMessage = sprintf("The selected .ini file does not contain a [%s] section", TempControlSection);
        errordlg(warningMessage, 'Invalid File');
        configLoadStatus = false;
        return
      end

      dlg.SharedProgressDlg = updateProgressDlg(dlg, 4/4, loadingMessage);

  end

  % Create and display the progress bar
  dlg.SharedProgressDlg = updateProgressDlg(dlg, 0, 'Awaiting response from Arduino...');

  sendPIDGains(dlg, 1/5);
  receivePIDGains(dlg, 2/5);

  sendControlParameters(dlg, 3/5);
  receiveControlParameters(dlg, 4/5);

  dlg.SharedProgressDlg = updateProgressDlg(dlg, 5/5, 'Finished communicating with Arduino');

  % Close the progress bar
  dlg = closeProgressDlg(dlg);

endfunction
