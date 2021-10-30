w %-*- texinfo -*-
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
            % Create and display the progress bar
            updateProgressDlg(dlg, 'Loading config file...');
            dlg.SharedProgressDlg.Title = 'Loading Config';

            % Create fully-formed filename as a string
            configFullPath = fullfile(configFilePath, configFileName);

            % Read the .ini file
            ini.ReadFile(configFullPath);

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
                if isvalid(dlg.SharedProgressDlg)
                    close(dlg.SharedProgressDlg);
                end

                warningMessage = sprintf("The selected .ini file does not contain a [%s] section", PIDSection);
                uialert(dlg.UIFigure, warningMessage, 'Invalid File');
                configLoadStatus = false;
                return
            end

            TempControlSection = 'Temperature Control';

            if ini.IsSections(TempControlSection)

                if ini.IsKeys(TempControlSection, 'startTemp')
                    dlg.Data.startTemp = ...
                        ini.GetValues(TempControlSection, 'startTemp');
                    dlg.StartTempCEditField.Value = dlg.Data.startTemp;
                end

                if ini.IsKeys(TempControlSection, 'endTemp')
                    dlg.Data.endTemp = ...
                        ini.GetValues(TempControlSection, 'endTemp');
                    dlg.EndTempCEditField.Value = dlg.Data.endTemp;
                end

                if ini.IsKeys(TempControlSection, 'rampUpRate')
                    dlg.Data.rampUpRate = ...
                        ini.GetValues(TempControlSection, 'rampUpRate');
                    dlg.RateCminEditField.Value = dlg.Data.rampUpRate;
                end

                if ini.IsKeys(TempControlSection, 'holdTime')
                    dlg.Data.holdTime = ...
                        ini.GetValues(TempControlSection, 'holdTime');
                    dlg.HoldTimesecEditField.Value = dlg.Data.holdTime;
                end

            else
                % Close the progress bar
                if isvalid(dlg.SharedProgressDlg)
                    close(dlg.SharedProgressDlg);
                end

                warningMessage = sprintf("The selected .ini file does not contain a [%s] section", TempControlSection);
                uialert(dlg.UIFigure, warningMessage, 'Invalid File');
                configLoadStatus = false;
                return
            end

    end

    % Create and display the progress bar
    updateProgressDlg(dlg, 'Awaiting response from Arduino...');

    sendPIDGains(dlg);
    receivePIDGains(dlg);

    sendControlParameters(dlg);
    receiveControlParameters(dlg);

    % Close the progress bar
    if isvalid(dlg.SharedProgressDlg)
        close(dlg.SharedProgressDlg);
    end

endfunction
