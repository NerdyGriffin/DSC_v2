classdef DSC_PID_Tuning_UI_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                      matlab.ui.Figure
        GridLayout                    matlab.ui.container.GridLayout
        LeftPanel                     matlab.ui.container.Panel
        GridLayout2                   matlab.ui.container.GridLayout
        SerialPortEditField           matlab.ui.control.EditField
        SerialPortEditFieldLabel      matlab.ui.control.Label
        SetSerialPortButton           matlab.ui.control.Button
        PIDParametersPanel            matlab.ui.container.Panel
        GridLayout7                   matlab.ui.container.GridLayout
        PIDAutotunerButton            matlab.ui.control.Button
        AutomatedKdSweepButton        matlab.ui.control.Button
        AutomatedKiSweepButton        matlab.ui.control.Button
        AbortSweepButton              matlab.ui.control.Button
        AutomatedKpSweepButton        matlab.ui.control.Button
        KdEditField                   matlab.ui.control.NumericEditField
        KdEditFieldLabel              matlab.ui.control.Label
        KiEditField                   matlab.ui.control.NumericEditField
        KiEditFieldLabel              matlab.ui.control.Label
        KpEditField                   matlab.ui.control.NumericEditField
        KpEditFieldLabel              matlab.ui.control.Label
        StopExperimentButton          matlab.ui.control.Button
        ApplyPIDParametersButton      matlab.ui.control.Button
        StartExperimentButton         matlab.ui.control.Button
        LoadConfigFileButton          matlab.ui.control.Button
        CenterPanel                   matlab.ui.container.Panel
        GridLayout3                   matlab.ui.container.GridLayout
        UIAxes2                       matlab.ui.control.UIAxes
        UIAxes                        matlab.ui.control.UIAxes
        RightPanel                    matlab.ui.container.Panel
        GridLayout4                   matlab.ui.container.GridLayout
        TestSampleLiveDataPanel       matlab.ui.container.Panel
        GridLayout6                   matlab.ui.container.GridLayout
        PWMDutyCycleEditField_2       matlab.ui.control.NumericEditField
        PWMDutyCycleEditField_2Label  matlab.ui.control.Label
        CurrentmAEditField_2          matlab.ui.control.NumericEditField
        CurrentmAEditField_2Label     matlab.ui.control.Label
        TemperatureCEditField_2       matlab.ui.control.NumericEditField
        TemperatureCEditField_2Label  matlab.ui.control.Label
        ReferenceSampleLiveDataPanel  matlab.ui.container.Panel
        GridLayout5                   matlab.ui.container.GridLayout
        PWMDutyCycleEditField         matlab.ui.control.NumericEditField
        PWMDutyCycleEditFieldLabel    matlab.ui.control.Label
        CurrentmAEditField            matlab.ui.control.NumericEditField
        CurrentmAEditFieldLabel       matlab.ui.control.Label
        TemperatureCEditField         matlab.ui.control.NumericEditField
        TemperatureCEditFieldLabel    matlab.ui.control.Label
        TargetTempCEditField          matlab.ui.control.NumericEditField
        TargetTempCEditFieldLabel     matlab.ui.control.Label
        ElapsedTimesecEditField       matlab.ui.control.NumericEditField
        ElapsedTimesecEditFieldLabel  matlab.ui.control.Label
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
        twoPanelWidth = 768;
    end


    %   DSC_v2: UI and control systems for prototype DSC system
    %       Copyright (C) 2020  Christian Kunis
    %
    %       This program is free software: you can redistribute it and/or modify
    %       it under the terms of the GNU General Public License as published by
    %       the Free Software Foundation, either version 3 of the License, or
    %       (at your option) any later version.
    %
    %       This program is distributed in the hope that it will be useful,
    %       but WITHOUT ANY WARRANTY; without even the implied warranty of
    %       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    %       GNU General Public License for more details.
    %
    %       You should have received a copy of the GNU General Public License
    %       along with this program. If not, see <https://www.gnu.org/licenses/>.
    %
    %       You may contact the author at ckunis.contact@gmail.com

    properties (Access = public)
        Arduino % The serialport object used for communications
        SerialPort % The name of the serial port to be used
        SerialPortList % The list of available serial ports

        BangOffLine
        TargetMaxLine
        TargetLine % Animate line object for the target temperature
        TargetMinLine
        BangOnLine
        RefSampleLine % Animated line object for the reference sample
        TestSampleLine % Animated line object for the test sample

        RefDutyCycleLine
        SampDutyCycleLine

        % The number of new samples to wait before attempting to update
        % the numeric fields
        DataRefreshDelay = 1;

        % The number of new samples to wait before attempting to refresh
        % the plots
        PlotRefreshDelay = 10;

        Data struct

        SharedProgressDlg matlab.ui.dialog.ProgressDialog

        AutomatedTestIsRunning

        PIDAutotunerIsRunning
    end

    methods (Access = public)

        function updateProgressDlg(app, message)
            if isvalid(app.SharedProgressDlg)
                app.SharedProgressDlg.Message = message;
            else
                % Create and display the progress bar
                app.SharedProgressDlg = uiprogressdlg(app.UIFigure,'Title','Communicating with Arduino', ...
                    'Message',message,'Indeterminate','on');
            end
            drawnow
        end

        function initializeSerialPort(app)
            % Get the list of available serial ports
            app.SerialPortList = serialportlist("available");
            disp('Available serial ports:')
            disp(app.SerialPortList)
            app.StartExperimentButton.Enable = 'off';

            if (isempty(app.SerialPortList))
                % Display a warning if no serial ports found
                message = 'No available serial ports were found. Make sure the arduino device is plugged into this computer via USB, and that it is not in use by another program (such as Arduino IDE).';
                uialert(app.UIFigure,message,'No Serial Device');
            else
                if (isempty(app.SerialPortEditField.Value))
                    % Set the default serial port to the last port in the list
                    app.SerialPortEditField.Value = app.SerialPortList(end);
                end

                if ~any(contains(app.SerialPortList, app.SerialPortEditField.Value))
                    message = sprintf("%s is not in the list of available serial ports", app.SerialPortEditField.Value);
                    uialert(app.UIFigure,message,'Invalid Serial Port')
                else
                    app.SerialPort = app.SerialPortEditField.Value;

                    if exist(app.Arduino,"var") && isvalid(app.Arduino)
                        delele(app.Arduino)
                    end
                    % Create an serial port object where you specify the USB port
                    % (look in Arduino->tools -> port and the baud rate (9600)
                    app.Arduino = serialport(app.SerialPort, 9600);

                    % Create and display the progress bar
                    updateProgressDlg(app, 'Awaiting response from Arduino...');

                    if isempty(readline(app.Arduino))
                        message = sprintf("There was no response from the device on '%s'. Make sure that this is the correct serial port, and that the 'dsc_arduino' sketch has been upload onto the Arduino.", app.SerialPort);
                        uialert(app.UIFigure,message,'Failed to communicate with Arduino');
                    else
                        % Request the temperature control parameters from the arduino
                        flush(app.Arduino);
                        write(app.Arduino, 'i', 'char');
                        receivePIDGains(app);
                        receiveControlParameters(app);
                        app.StartExperimentButton.Enable = 'on';
                    end

                    % Close the progress bar
                    if isvalid(app.SharedProgressDlg)
                        close(app.SharedProgressDlg)
                    end
                end
            end
        end

        function configLoadStatus = loadConfigFile(app)
            %   Load control parameters from a .ini file

            ini = IniConfig();

            % Prompt the user to select a file
            [configFileName, configFilePath] = uigetfile('*.ini');

            switch configFileName
                case 0
                    % Cancel the read operation and return an empty array
                    % if the user closes the file selection window
                    configLoadStatus = false;
                    return

                otherwise
                    % Create and display the progress bar
                    updateProgressDlg(app, 'Loading config file...');
                    app.SharedProgressDlg.Title = 'Loading Config';

                    % Create fully-formed filename as a string
                    configFullPath = fullfile(configFilePath, configFileName);

                    % Read the .ini file
                    ini.ReadFile(configFullPath)

                    PIDSection = 'PID Settings';
                    if ini.IsSections(PIDSection)
                        if ini.IsKeys(PIDSection,'Kp')
                            app.KpEditField.Value = ...
                                ini.GetValues('PID Settings','Kp');
                        end

                        if ini.IsKeys(PIDSection,'Ki')
                            app.KiEditField.Value = ...
                                ini.GetValues('PID Settings','Ki');
                        end

                        if ini.IsKeys(PIDSection,'Kd')
                            app.KdEditField.Value = ...
                                ini.GetValues('PID Settings','Kd');
                        end

                    else
                        % Close the progress bar
                        if isvalid(app.SharedProgressDlg)
                            close(app.SharedProgressDlg)
                        end

                        warningMessage = sprintf("The selected .ini file does not contain a [%s] section", PIDSection);
                        uialert(app.UIFigure,warningMessage,'Invalid File');
                        configLoadStatus = false;
                        return
                    end

                    TempControlSection = 'Temperature Control';
                    if ini.IsSections(TempControlSection)
                        if ini.IsKeys(TempControlSection,'startTemp')
                            app.Data.startTemp = ...
                                ini.GetValues(TempControlSection,'startTemp');
                        end

                        if ini.IsKeys(TempControlSection,'endTemp')
                            app.Data.endTemp = ...
                                ini.GetValues(TempControlSection,'endTemp');
                        end

                        if ini.IsKeys(TempControlSection,'rampUpRate')
                            app.Data.rampUpRate = ...
                                ini.GetValues(TempControlSection,'rampUpRate');
                        end

                        if ini.IsKeys(TempControlSection,'holdTime')
                            app.Data.holdTime = ...
                                ini.GetValues(TempControlSection,'holdTime');
                        end

                    else
                        % Close the progress bar
                        if isvalid(app.SharedProgressDlg)
                            close(app.SharedProgressDlg)
                        end

                        warningMessage = sprintf("The selected .ini file does not contain a [%s] section", TempControlSection);
                        uialert(app.UIFigure,warningMessage,'Invalid File');
                        configLoadStatus = false;
                        return
                    end
            end

            % Close the progress bar
            if isvalid(app.SharedProgressDlg)
                close(app.SharedProgressDlg)
            end
        end

        function automatedPIDSweep(app, sweepType)
            app.AbortSweepButton.Enable = 'on';
            app.AutomatedKpSweepButton.Enable = 'off';
            app.AutomatedKiSweepButton.Enable = 'off';
            app.AutomatedKdSweepButton.Enable = 'off';
            app.ApplyPIDParametersButton.Enable = 'off';
            app.LoadConfigFileButton.Enable = 'off';
            app.StartExperimentButton.Enable = 'off';

            Kp = app.KpEditField.Value;
            Ki = app.KiEditField.Value;
            Kd = app.KdEditField.Value;

            switch sweepType
                case 'P'
                    sweepVal = Kp;
                case 'I'
                    sweepVal = Ki;
                case 'D'
                    sweepVal = Kd;
                otherwise
                    message = sprintf("Invalid sweepType: '%s'\nPlease report this bug to the developer.", sweepType);
                    uialert(app.UIFigure,message,'Internal Error');
                    app.AutomatedTestIsRunning = false;
                    return
            end

            n = floor(log10(sweepVal));
            if n < -1
                kMin = 1e-2;
                kStep = 1e-2;
                kMax = 1e-1;
            else
                kMin = (10^n);
                kStep = (10^(n-1));
                kMax = (10^(n+1));

            end

            app.AutomatedTestIsRunning = true;
            for kVar = kMin:kStep:kMax
                switch sweepType
                    case 'P'
                        Kp = kVar;
                    case 'I'
                        Ki = kVar;
                    case 'D'
                        Kd = kVar;
                    otherwise
                        message = sprintf("Invalid sweepType: '%s'\nPlease report this bug to the developer.", sweepType);
                        uialert(app.UIFigure,message,'Internal Error');
                        app.AutomatedTestIsRunning = false;
                        return
                end

                % Set the PID gain values for the next trial
                app.KpEditField.Value = Kp;
                app.KiEditField.Value = Ki;
                app.KdEditField.Value = Kd;

                % Create and display the progress bar
                updateProgressDlg(app, 'Awaiting response from Arduino...');

                % Sync PID gains with Arduino
                sendPIDGains(app);
                receivePIDGains(app);

                % Set the control parameters for PID tuning
                app.Data.startTemp = 30;
                app.Data.endTemp = 120;
                app.Data.rampUpRate = 20;
                app.Data.holdTime = 0;

                % Sync control parameters with arduino
                sendControlParameters(app);
                receiveControlParameters(app);

                % Close the progress bar
                if isvalid(app.SharedProgressDlg)
                    close(app.SharedProgressDlg)
                end

                % Start the experiment
                startExperiment(app);

                if ~app.AutomatedTestIsRunning
                    app.AutomatedTestIsRunning = false;
                    break
                end

                pauseDuration = 5*60; % Duration in minutes
                d = uiprogressdlg(app.UIFigure,'Title','Please Wait while the samples cool to room temperature.', ...
                    'Message','Time remaining: ','Cancelable','on');
                drawnow

                tStart = tic;
                pause(0.5)
                tProgress = toc(tStart);
                while tProgress < pauseDuration
                    % Check for Cancel button press
                    if d.CancelRequested
                        app.AutomatedTestIsRunning = false;
                        break
                    end
                    % Update the progress bar, report time remaining
                    d.Value = tProgress/pauseDuration;
                    d.Message = sprintf("Time remaining: %s (approximate).", datestr(seconds(pauseDuration-tProgress),'HH:MM:SS'));
                    pause(0.5);
                    tProgress = toc(tStart);
                end

                % Close dialog box
                close(d)

                if ~app.AutomatedTestIsRunning
                    app.AutomatedTestIsRunning = false;
                    break
                end
            end

            app.AutomatedKpSweepButton.Enable = 'on';
            app.AutomatedKiSweepButton.Enable = 'on';
            app.AutomatedKdSweepButton.Enable = 'on';
            app.AbortSweepButton.Enable = 'off';
            app.AutomatedTestIsRunning = false;
        end

        function startPIDAutotuner(app)
            flush(app.Arduino);
            write(app.Arduino, 'a', 'char');

            awaitStart = true;
            while awaitStart
                serialData = strip(readline(app.Arduino));
                if strlength(serialData) == 1
                    switch strip(serialData)
                        case 'a'
                            app.PIDAutotunerIsRunning = true;
                            setRunningUI(app);
                            receiveSerialData(app);
                            awaitStart = false;
                        case 'x'
                            setIdleUI(app);
                            disp('Received end signal')
                            awaitStart = false;
                        otherwise
                            disp('Unrecognized data flag while awaiting start response:')
                            disp(serialData);
                    end
                end
            end
        end

        function startExperiment(app)
            flush(app.Arduino);
            write(app.Arduino, 's', 'char');

            awaitStart = true;
            while awaitStart
                serialData = strip(readline(app.Arduino));
                if strlength(serialData) == 1
                    switch strip(serialData)
                        case 's'
                            setRunningUI(app);
                            receiveSerialData(app);
                            awaitStart = false;
                        case 'x'
                            setIdleUI(app);
                            disp('Received end signal')
                            awaitStart = false;
                        otherwise
                            disp('Unrecognized data flag while awaiting start response:')
                            disp(serialData);
                    end
                end
            end
        end

        function sendPIDGains(app)
            updateProgressDlg(app, 'Sending PID Gains to Arduino...');

            % Send the PID gain constants via the serial bus
            flush(app.Arduino);
            write(app.Arduino, 'p', 'char');

            write(app.Arduino, string(app.KpEditField.Value), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.KiEditField.Value), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.KdEditField.Value), 'string');
        end

        function receivePIDGains(app)
            % Receive the PID gain constants via the serial bus
            updateProgressDlg(app, 'Receiving PID Gains from Arduino...');
            awaitResponse = true;
            while awaitResponse
                serialData = strip(readline(app.Arduino));
                if strlength(serialData) == 1
                    switch strip(serialData)
                        case 'k'
                            readline(app.Arduino);
                            serialData = strip(readline(app.Arduino));
                            disp(serialData)
                            [parsedData, dataIsNum] = str2num(serialData);
                            if dataIsNum && length(parsedData) == 3
                                app.KpEditField.Value = parsedData(1); %double(readline(app.Arduino));
                                app.KiEditField.Value = parsedData(2); %double(readline(app.Arduino));
                                app.KdEditField.Value = parsedData(3); %double(readline(app.Arduino));
                            end
                            awaitResponse = false;
                        case 'x'
                            setIdleUI(app);
                            disp('Received end signal')
                            awaitResponse = false;
                        otherwise
                            disp('Unrecognized data flag while awaiting PID Gains:')
                            disp(serialData);
                    end
                end
            end
        end

        function sendControlParameters(app)
            updateProgressDlg(app, 'Sending temperature control parameters to Arduino...');

            flush(app.Arduino);
            write(app.Arduino, 'l', 'char');

            write(app.Arduino, string(app.Data.startTemp), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.Data.endTemp), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.Data.rampUpRate), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.Data.holdTime), 'string');
        end

        function receiveControlParameters(app)
            updateProgressDlg(app, 'Receiving temperature control parameters from Arduino...');
            awaitResponse = true;
            while awaitResponse
                serialData = strip(readline(app.Arduino));
                if strlength(serialData) == 1
                    switch strip(serialData)
                        case 'c'
                            readline(app.Arduino);
                            serialData = strip(readline(app.Arduino));
                            disp(serialData)
                            [parsedData, dataIsNum] = str2num(serialData);
                            if dataIsNum && length(parsedData) == 4
                                app.Data.startTemp = parsedData(1); %double(readline(app.Arduino));
                                app.Data.endTemp = parsedData(2); %double(readline(app.Arduino));
                                app.Data.rampUpRate = parsedData(3); %double(readline(app.Arduino));
                                app.Data.holdTime = parsedData(4); %double(readline(app.Arduino));
                            end
                            awaitResponse = false;
                        case 'x'
                            setIdleUI(app);
                            disp('Received end signal')
                            awaitResponse = false;
                        otherwise
                            disp('Unrecognized data flag while awaiting control params:')
                            disp(serialData);
                    end
                end
            end
        end

        function receiveSerialData(app)
            updateProgressDlg(app, 'Awaiting initial data...');

            app.Data.Kp = app.KpEditField.Value;
            app.Data.Ki = app.KiEditField.Value;
            app.Data.Kd = app.KdEditField.Value;

            app.Data.startDateTime = datetime;

            if not(isfolder('autosave'))
                mkdir('autosave');
            end
            currentDataString = datestr(app.Data.startDateTime, 'yyyy-mm-dd-HHMM');
            if app.AutomatedTestIsRunning
                Kp_str = strrep(num2str(app.KpEditField.Value),'.','');
                matfileName = ['autosave/autoPIDTestData-',Kp_str,'-P-',currentDataString,'.mat'];
            else
                matfileName = ['autosave/autoSaveData-',currentDataString,'.mat'];
            end

            app.Data.elapsedTime = zeros(1,app.PlotRefreshDelay);
            app.Data.targetTemp = zeros(1,app.PlotRefreshDelay);
            app.Data.refTemp = zeros(1,app.PlotRefreshDelay);
            app.Data.sampTemp = zeros(1,app.PlotRefreshDelay);
            app.Data.refCurrent = zeros(1,app.PlotRefreshDelay);
            app.Data.sampCurrent = zeros(1,app.PlotRefreshDelay);
            app.Data.refHeatFlow = zeros(1,app.PlotRefreshDelay);
            app.Data.sampHeatFlow = zeros(1,app.PlotRefreshDelay);
            app.Data.refDutyCycle = zeros(1,app.PlotRefreshDelay);
            app.Data.sampDutyCycle = zeros(1,app.PlotRefreshDelay);

            dataLength = 0;

            experimentIsRunning = true;
            while experimentIsRunning
                serialData = strip(readline(app.Arduino));
                if strlength(serialData) == 1
                    switch strip(serialData)
                        case 'x'
                            experimentIsRunning = false;
                            disp('Received end signal')
                            updateProgressDlg(app, 'Awaiting response from Arduino...');
                            receivePIDGains(app);
                            receiveControlParameters(app);
                        otherwise
                            disp('Unrecognized data flag while awaiting data:')
                            disp(serialData)
                    end
                else
                    [parsedData, dataIsNum] = str2num(serialData);
                    if dataIsNum && length(parsedData) == 10
                        dataLength = dataLength + 1;
                        app.Data.elapsedTime(dataLength) = parsedData(1);
                        app.Data.targetTemp(dataLength) = parsedData(2);
                        app.Data.refTemp(dataLength) = parsedData(3);
                        app.Data.sampTemp(dataLength) = parsedData(4);
                        app.Data.refCurrent(dataLength) = parsedData(5);
                        app.Data.sampCurrent(dataLength) = parsedData(6);
                        app.Data.refHeatFlow(dataLength) = parsedData(7);
                        app.Data.sampHeatFlow(dataLength) = parsedData(8);
                        app.Data.refDutyCycle(dataLength) = parsedData(9);
                        app.Data.sampDutyCycle(dataLength) = parsedData(10);

                        if ~mod(dataLength, app.DataRefreshDelay)
                            updateLiveData(app, ...
                                app.Data.elapsedTime(dataLength), ...
                                app.Data.targetTemp(dataLength), ...
                                app.Data.refTemp(dataLength), ...
                                app.Data.sampTemp(dataLength), ...
                                app.Data.refCurrent(dataLength), ...
                                app.Data.sampCurrent(dataLength), ...
                                app.Data.refDutyCycle(dataLength), ...
                                app.Data.sampDutyCycle(dataLength));
                        end

                        if ~mod(dataLength, app.PlotRefreshDelay)
                            refreshLivePlot(app, ...
                                app.Data.elapsedTime, app.Data.targetTemp, ...
                                app.Data.refTemp, app.Data.sampTemp, ...
                                app.Data.refDutyCycle, app.Data.sampDutyCycle);
                            % Close the progress bar
                            if isvalid(app.SharedProgressDlg)
                                close(app.SharedProgressDlg)
                            end
                        end
                    else
                        disp(parsedData)
                    end
                end
            end

            app.Data.dataLength = dataLength;

            if app.PIDAutotunerIsRunning
                % Recieve the PID gains which are sent automatically after
                % the autotuner is complete.
                disp('PID Autotuner results:')
                receivePIDGains(app);
                app.PIDAutotunerIsRunning = false;
                % This also updates the app.Data struct so that the new
                % PID gains will be included in the autosave file.
            end

            % Close the progress bar
            if isvalid(app.SharedProgressDlg)
                close(app.SharedProgressDlg)
            end

            saveData = app.Data;
            save(matfileName,'-struct','saveData')
            if isfile(matfileName)
                beep
                message = sprintf("Autosave file created: '%s'\n", matfileName);
                disp(message)
                uialert(app.UIFigure,message,'Autosave Successful','Icon','success');
            end

            updateLiveData(app, ...
                app.Data.elapsedTime(dataLength), ...
                app.Data.targetTemp(dataLength), ...
                app.Data.refTemp(dataLength), ...
                app.Data.sampTemp(dataLength), ...
                app.Data.refCurrent(dataLength), ...
                app.Data.sampCurrent(dataLength), ...
                app.Data.refDutyCycle(dataLength), ...
                app.Data.sampDutyCycle(dataLength));

            refreshLivePlot(app, app.Data.elapsedTime, app.Data.targetTemp, ...
                app.Data.refTemp, app.Data.sampTemp, ...
                app.Data.refDutyCycle, app.Data.sampDutyCycle);

            setIdleUI(app);
        end

        function setRunningUI(app)
            app.StartExperimentButton.Enable = 'off';
            app.StopExperimentButton.Enable = 'on';
            app.LoadConfigFileButton.Enable = 'off';
            app.ApplyPIDParametersButton.Enable = 'off';
            app.KpEditField.Editable = 'off';
            app.KiEditField.Editable = 'off';
            app.KdEditField.Editable = 'off';
            app.AutomatedKpSweepButton.Enable = 'off';
            app.AutomatedKiSweepButton.Enable = 'off';
            app.AutomatedKdSweepButton.Enable = 'off';
            app.AbortSweepButton.Enable = 'off';
            app.PIDAutotunerButton.Enable = 'off';
            app.SetSerialPortButton.Enable = 'off';
            app.SerialPortEditField.Editable = 'off';
        end

        function setIdleUI(app)
            app.StartExperimentButton.Enable = 'on';
            app.StopExperimentButton.Enable = 'off';
            app.LoadConfigFileButton.Enable = 'on';
            app.ApplyPIDParametersButton.Enable = 'on';
            app.KpEditField.Editable = 'on';
            app.KiEditField.Editable = 'on';
            app.KdEditField.Editable = 'on';
            app.AutomatedKpSweepButton.Enable = 'on';
            app.AutomatedKiSweepButton.Enable = 'on';
            app.AutomatedKdSweepButton.Enable = 'on';
            app.AbortSweepButton.Enable = 'off';
            app.PIDAutotunerButton.Enable = 'on';
            app.SetSerialPortButton.Enable = 'on';
            app.SerialPortEditField.Editable = 'on';
        end

        function updateLiveData(app, elapsedTime, targetTemp, ...
                refTemp, sampTemp, refCurrent, sampCurrent, ...
                refDutyCycle, sampDutyCycle)
            drawnow limitrate nocallbacks

            % Convert from milliseconds to seconds
            app.ElapsedTimesecEditField.Value = elapsedTime;

            app.TargetTempCEditField.Value = targetTemp;

            app.TemperatureCEditField.Value = refTemp;
            app.CurrentmAEditField.Value = refCurrent;
            app.PWMDutyCycleEditField.Value = refDutyCycle;

            app.TemperatureCEditField_2.Value = sampTemp;
            app.CurrentmAEditField_2.Value = sampCurrent;
            app.PWMDutyCycleEditField_2.Value = sampDutyCycle;

            drawnow limitrate
        end

        function refreshLivePlot(app, elapsedTimeArray, ...
                targetTempArray, refTempArray, sampTempArray, ...
                refDutyCycleArray, sampDutyCycleArray)

            drawnow limitrate nocallbacks

            clearpoints(app.BangOffLine)
            clearpoints(app.TargetMaxLine)
            clearpoints(app.TargetLine)
            clearpoints(app.TargetMinLine)
            clearpoints(app.BangOnLine)
            clearpoints(app.RefSampleLine)
            clearpoints(app.TestSampleLine)

            clearpoints(app.RefDutyCycleLine)
            clearpoints(app.SampDutyCycleLine)

            % When the temperature is less than {TargetTemp - BANG_RANGE},
            % the PID control is deactivated, and the output is set to max
            BANG_RANGE = 20;
            MINIMUM_ACCEPTABLE_ERROR = 5;

            % Update the plots
            addpoints(app.BangOffLine, elapsedTimeArray, targetTempArray + BANG_RANGE)
            addpoints(app.TargetMaxLine, elapsedTimeArray, targetTempArray + MINIMUM_ACCEPTABLE_ERROR)
            addpoints(app.TargetLine, elapsedTimeArray, targetTempArray)
            addpoints(app.TargetMinLine, elapsedTimeArray, targetTempArray - MINIMUM_ACCEPTABLE_ERROR)
            addpoints(app.BangOnLine, elapsedTimeArray, targetTempArray - BANG_RANGE)
            addpoints(app.RefSampleLine, elapsedTimeArray, refTempArray)
            addpoints(app.TestSampleLine, elapsedTimeArray, sampTempArray)

            addpoints(app.RefDutyCycleLine, elapsedTimeArray, refDutyCycleArray)
            addpoints(app.SampDutyCycleLine, elapsedTimeArray, sampDutyCycleArray)

            legend(app.UIAxes, 'Location', 'best')

            legend(app.UIAxes2, 'Location', 'best')

            ylim(app.UIAxes2, [0 1])

            drawnow limitrate
        end
    end


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            % Initialize the struct to prevent errors
            app.Data = struct('Kp', 0, 'Ki', 0, 'Kd', 0, ...
                'startTemp', 0, 'endTemp', 0, 'rampUpRate', 0, ...
                'holdTime', 0, 'startDateTime', datetime);

            app.SerialPortEditField.Value = '';

            initializeSerialPort(app);

            % Create the animatedline objects
            app.BangOffLine = animatedline(app.UIAxes, 'Color', 'yellow', ...
                'LineStyle', '-.');
            app.TargetMaxLine = animatedline(app.UIAxes, 'Color', 'green', ...
                'LineStyle', '--');
            app.TargetLine = animatedline(app.UIAxes, 'Color', 'black', ...
                'LineStyle', ':');
            app.TargetMinLine = animatedline(app.UIAxes, 'Color', 'green', ...
                'LineStyle', '--');
            app.BangOnLine = animatedline(app.UIAxes, 'Color', 'yellow', ...
                'LineStyle', '-.');
            app.RefSampleLine = animatedline(app.UIAxes, 'Color', 'blue', ...
                'LineStyle', '-');
            app.TestSampleLine = animatedline(app.UIAxes, 'Color', 'red', ...
                'LineStyle', '-');

            app.RefDutyCycleLine = animatedline(app.UIAxes2, 'Color', 'blue', ...
                'LineStyle', '-');
            app.SampDutyCycleLine = animatedline(app.UIAxes2, 'Color', 'red', ...
                'LineStyle', '-');

            % Create a legend for the temperature plot
            legend(app.UIAxes, 'PID Upper Bound', 'Target Upper Bound', ...
                'Target Temperature', 'Target Lower Bound', ...
                'PID Lower Bound', 'Reference Sample', 'Test Sample', ...
                'Location', 'best')

            % Create a legend for the duty cycle plot
            legend(app.UIAxes2, 'Reference Sample', 'Test Sample', ...
                'Location', 'best')

            setIdleUI(app);
        end

        % Changes arrangement of the app based on UIFigure width
        function updateAppLayout(app, event)
            currentFigureWidth = app.UIFigure.Position(3);
            if(currentFigureWidth <= app.onePanelWidth)
                % Change to a 3x1 grid
                app.GridLayout.RowHeight = {480, 480, 480};
                app.GridLayout.ColumnWidth = {'1x'};
                app.CenterPanel.Layout.Row = 1;
                app.CenterPanel.Layout.Column = 1;
                app.LeftPanel.Layout.Row = 2;
                app.LeftPanel.Layout.Column = 1;
                app.RightPanel.Layout.Row = 3;
                app.RightPanel.Layout.Column = 1;
            elseif (currentFigureWidth > app.onePanelWidth && currentFigureWidth <= app.twoPanelWidth)
                % Change to a 2x2 grid
                app.GridLayout.RowHeight = {480, 480};
                app.GridLayout.ColumnWidth = {'1x', '1x'};
                app.CenterPanel.Layout.Row = 1;
                app.CenterPanel.Layout.Column = [1,2];
                app.LeftPanel.Layout.Row = 2;
                app.LeftPanel.Layout.Column = 1;
                app.RightPanel.Layout.Row = 2;
                app.RightPanel.Layout.Column = 2;
            else
                % Change to a 1x3 grid
                app.GridLayout.RowHeight = {'1x'};
                app.GridLayout.ColumnWidth = {220, '1x', 220};
                app.LeftPanel.Layout.Row = 1;
                app.LeftPanel.Layout.Column = 1;
                app.CenterPanel.Layout.Row = 1;
                app.CenterPanel.Layout.Column = 2;
                app.RightPanel.Layout.Row = 1;
                app.RightPanel.Layout.Column = 3;
            end
        end

        % Button pushed function: StartExperimentButton
        function StartExperimentButtonPushed(app, event)
            app.StartExperimentButton.Enable = 'off';

            startExperiment(app);
        end

        % Button pushed function: StopExperimentButton
        function StopExperimentButtonPushed(app, event)
            app.StopExperimentButton.Enable = 'off';

            write(app.Arduino, 'x', 'char');
        end

        % Button pushed function: LoadConfigFileButton
        function LoadConfigFileButtonPushed(app, event)
            app.LoadConfigFileButton.Enable = 'off';

            loadConfigFile(app);

            % Create and display the progress bar
            updateProgressDlg(app, 'Awaiting response from Arduino...');

            sendPIDGains(app);
            receivePIDGains(app);

            sendControlParameters(app);
            receiveControlParameters(app);

            % Close the progress bar
            if isvalid(app.SharedProgressDlg)
                close(app.SharedProgressDlg)
            end

            app.LoadConfigFileButton.Enable = 'on';
        end

        % Button pushed function: ApplyPIDParametersButton
        function ApplyPIDParametersButtonPushed(app, event)
            app.ApplyPIDParametersButton.Enable = 'off';

            % Create and display the progress bar
            updateProgressDlg(app, 'Awaiting response from Arduino...');

            sendPIDGains(app);
            receivePIDGains(app);

            sendControlParameters(app);
            receiveControlParameters(app);

            % Close the progress bar
            if isvalid(app.SharedProgressDlg)
                close(app.SharedProgressDlg)
            end

            app.ApplyPIDParametersButton.Enable = 'on';
        end

        % Button pushed function: SetSerialPortButton
        function SetSerialPortButtonPushed(app, event)
            app.SetSerialPortButton.Enable = 'off';

            initializeSerialPort(app)

            app.SetSerialPortButton.Enable = 'on';
        end

        % Close request function: UIFigure
        function UIFigureCloseRequest(app, event)
            delete(app)
        end

        % Button pushed function: AutomatedKpSweepButton
        function AutomatedKpSweepButtonPushed(app, event)
            app.AutomatedKdSweepButton.Enable = 'off';

            automatedPIDSweep(app, 'P');
        end

        % Button pushed function: AbortSweepButton
        function AbortSweepButtonPushed(app, event)
            app.AbortSweepButton.Enable = 'off';
            app.StopExperimentButton.Enable = 'off';

            app.AutomatedTestIsRunning = false;

            write(app.Arduino, 'x', 'char');

            app.AbortSweepButton.Enable = 'on';
        end

        % Button pushed function: AutomatedKiSweepButton
        function AutomatedKiSweepButtonPushed(app, event)
            app.AutomatedKdSweepButton.Enable = 'off';

            automatedPIDSweep(app, 'I');
        end

        % Button pushed function: AutomatedKdSweepButton
        function AutomatedKdSweepButtonPushed(app, event)
            app.AutomatedKdSweepButton.Enable = 'off';

            automatedPIDSweep(app, 'D');
        end

        % Button pushed function: PIDAutotunerButton
        function PIDAutotunerButtonPushed(app, event)
            app.PIDAutotunerButton.Enable = 'off';

            startPIDAutotuner(app);
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [100 100 860 480];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.CloseRequestFcn = createCallbackFcn(app, @UIFigureCloseRequest, true);
            app.UIFigure.SizeChangedFcn = createCallbackFcn(app, @updateAppLayout, true);

            % Create GridLayout
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {220, '1x', 220};
            app.GridLayout.RowHeight = {'1x'};
            app.GridLayout.ColumnSpacing = 0;
            app.GridLayout.RowSpacing = 0;
            app.GridLayout.Padding = [0 0 0 0];
            app.GridLayout.Scrollable = 'on';

            % Create LeftPanel
            app.LeftPanel = uipanel(app.GridLayout);
            app.LeftPanel.Layout.Row = 1;
            app.LeftPanel.Layout.Column = 1;

            % Create GridLayout2
            app.GridLayout2 = uigridlayout(app.LeftPanel);
            app.GridLayout2.ColumnWidth = {'2x', '1x'};
            app.GridLayout2.RowHeight = {'1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x'};

            % Create LoadConfigFileButton
            app.LoadConfigFileButton = uibutton(app.GridLayout2, 'push');
            app.LoadConfigFileButton.ButtonPushedFcn = createCallbackFcn(app, @LoadConfigFileButtonPushed, true);
            app.LoadConfigFileButton.BackgroundColor = [0.302 0.7451 0.9333];
            app.LoadConfigFileButton.Layout.Row = 3;
            app.LoadConfigFileButton.Layout.Column = [1 2];
            app.LoadConfigFileButton.Text = 'Load Config File';

            % Create StartExperimentButton
            app.StartExperimentButton = uibutton(app.GridLayout2, 'push');
            app.StartExperimentButton.ButtonPushedFcn = createCallbackFcn(app, @StartExperimentButtonPushed, true);
            app.StartExperimentButton.BackgroundColor = [0 1 0];
            app.StartExperimentButton.Layout.Row = 1;
            app.StartExperimentButton.Layout.Column = [1 2];
            app.StartExperimentButton.Text = 'Start Experiment';

            % Create ApplyPIDParametersButton
            app.ApplyPIDParametersButton = uibutton(app.GridLayout2, 'push');
            app.ApplyPIDParametersButton.ButtonPushedFcn = createCallbackFcn(app, @ApplyPIDParametersButtonPushed, true);
            app.ApplyPIDParametersButton.BackgroundColor = [0 1 1];
            app.ApplyPIDParametersButton.Layout.Row = 4;
            app.ApplyPIDParametersButton.Layout.Column = [1 2];
            app.ApplyPIDParametersButton.Text = 'Apply PID Parameters';

            % Create StopExperimentButton
            app.StopExperimentButton = uibutton(app.GridLayout2, 'push');
            app.StopExperimentButton.ButtonPushedFcn = createCallbackFcn(app, @StopExperimentButtonPushed, true);
            app.StopExperimentButton.BackgroundColor = [1 0 0];
            app.StopExperimentButton.Layout.Row = 2;
            app.StopExperimentButton.Layout.Column = [1 2];
            app.StopExperimentButton.Text = 'Stop Experiment';

            % Create PIDParametersPanel
            app.PIDParametersPanel = uipanel(app.GridLayout2);
            app.PIDParametersPanel.Title = 'PID Parameters';
            app.PIDParametersPanel.Layout.Row = [5 8];
            app.PIDParametersPanel.Layout.Column = [1 2];

            % Create GridLayout7
            app.GridLayout7 = uigridlayout(app.PIDParametersPanel);
            app.GridLayout7.ColumnWidth = {'1x', '1x', '1x'};
            app.GridLayout7.RowHeight = {'1x', '1x', '1x', '1x'};

            % Create KpEditFieldLabel
            app.KpEditFieldLabel = uilabel(app.GridLayout7);
            app.KpEditFieldLabel.HorizontalAlignment = 'right';
            app.KpEditFieldLabel.Layout.Row = 1;
            app.KpEditFieldLabel.Layout.Column = 2;
            app.KpEditFieldLabel.Text = 'Kp';

            % Create KpEditField
            app.KpEditField = uieditfield(app.GridLayout7, 'numeric');
            app.KpEditField.Limits = [0 Inf];
            app.KpEditField.ValueDisplayFormat = '%.2f';
            app.KpEditField.Layout.Row = 1;
            app.KpEditField.Layout.Column = 3;

            % Create KiEditFieldLabel
            app.KiEditFieldLabel = uilabel(app.GridLayout7);
            app.KiEditFieldLabel.HorizontalAlignment = 'right';
            app.KiEditFieldLabel.Layout.Row = 2;
            app.KiEditFieldLabel.Layout.Column = 2;
            app.KiEditFieldLabel.Text = 'Ki';

            % Create KiEditField
            app.KiEditField = uieditfield(app.GridLayout7, 'numeric');
            app.KiEditField.Limits = [0 Inf];
            app.KiEditField.ValueDisplayFormat = '%.2f';
            app.KiEditField.Layout.Row = 2;
            app.KiEditField.Layout.Column = 3;

            % Create KdEditFieldLabel
            app.KdEditFieldLabel = uilabel(app.GridLayout7);
            app.KdEditFieldLabel.HorizontalAlignment = 'right';
            app.KdEditFieldLabel.Layout.Row = 3;
            app.KdEditFieldLabel.Layout.Column = 2;
            app.KdEditFieldLabel.Text = 'Kd';

            % Create KdEditField
            app.KdEditField = uieditfield(app.GridLayout7, 'numeric');
            app.KdEditField.Limits = [0 Inf];
            app.KdEditField.ValueDisplayFormat = '%.2f';
            app.KdEditField.Layout.Row = 3;
            app.KdEditField.Layout.Column = 3;

            % Create AutomatedKpSweepButton
            app.AutomatedKpSweepButton = uibutton(app.GridLayout7, 'push');
            app.AutomatedKpSweepButton.ButtonPushedFcn = createCallbackFcn(app, @AutomatedKpSweepButtonPushed, true);
            app.AutomatedKpSweepButton.WordWrap = 'on';
            app.AutomatedKpSweepButton.BackgroundColor = [0.3922 0.8314 0.0745];
            app.AutomatedKpSweepButton.Layout.Row = 1;
            app.AutomatedKpSweepButton.Layout.Column = 1;
            app.AutomatedKpSweepButton.Text = 'Automated Kp Sweep';

            % Create AbortSweepButton
            app.AbortSweepButton = uibutton(app.GridLayout7, 'push');
            app.AbortSweepButton.ButtonPushedFcn = createCallbackFcn(app, @AbortSweepButtonPushed, true);
            app.AbortSweepButton.WordWrap = 'on';
            app.AbortSweepButton.BackgroundColor = [0.851 0.3255 0.098];
            app.AbortSweepButton.Layout.Row = 4;
            app.AbortSweepButton.Layout.Column = 1;
            app.AbortSweepButton.Text = 'Abort Sweep';

            % Create AutomatedKiSweepButton
            app.AutomatedKiSweepButton = uibutton(app.GridLayout7, 'push');
            app.AutomatedKiSweepButton.ButtonPushedFcn = createCallbackFcn(app, @AutomatedKiSweepButtonPushed, true);
            app.AutomatedKiSweepButton.WordWrap = 'on';
            app.AutomatedKiSweepButton.BackgroundColor = [0.3922 0.8314 0.0745];
            app.AutomatedKiSweepButton.Layout.Row = 2;
            app.AutomatedKiSweepButton.Layout.Column = 1;
            app.AutomatedKiSweepButton.Text = 'Automated Ki Sweep';

            % Create AutomatedKdSweepButton
            app.AutomatedKdSweepButton = uibutton(app.GridLayout7, 'push');
            app.AutomatedKdSweepButton.ButtonPushedFcn = createCallbackFcn(app, @AutomatedKdSweepButtonPushed, true);
            app.AutomatedKdSweepButton.WordWrap = 'on';
            app.AutomatedKdSweepButton.BackgroundColor = [0.3922 0.8314 0.0745];
            app.AutomatedKdSweepButton.Layout.Row = 3;
            app.AutomatedKdSweepButton.Layout.Column = 1;
            app.AutomatedKdSweepButton.Text = 'Automated Kd Sweep';

            % Create PIDAutotunerButton
            app.PIDAutotunerButton = uibutton(app.GridLayout7, 'push');
            app.PIDAutotunerButton.ButtonPushedFcn = createCallbackFcn(app, @PIDAutotunerButtonPushed, true);
            app.PIDAutotunerButton.BackgroundColor = [0 1 0];
            app.PIDAutotunerButton.Layout.Row = 4;
            app.PIDAutotunerButton.Layout.Column = [2 3];
            app.PIDAutotunerButton.Text = 'PID Autotuner';

            % Create SetSerialPortButton
            app.SetSerialPortButton = uibutton(app.GridLayout2, 'push');
            app.SetSerialPortButton.ButtonPushedFcn = createCallbackFcn(app, @SetSerialPortButtonPushed, true);
            app.SetSerialPortButton.Layout.Row = 9;
            app.SetSerialPortButton.Layout.Column = [1 2];
            app.SetSerialPortButton.Text = 'Set Serial Port';

            % Create SerialPortEditFieldLabel
            app.SerialPortEditFieldLabel = uilabel(app.GridLayout2);
            app.SerialPortEditFieldLabel.HorizontalAlignment = 'right';
            app.SerialPortEditFieldLabel.Layout.Row = 10;
            app.SerialPortEditFieldLabel.Layout.Column = 1;
            app.SerialPortEditFieldLabel.Text = 'Serial Port';

            % Create SerialPortEditField
            app.SerialPortEditField = uieditfield(app.GridLayout2, 'text');
            app.SerialPortEditField.Layout.Row = 10;
            app.SerialPortEditField.Layout.Column = 2;

            % Create CenterPanel
            app.CenterPanel = uipanel(app.GridLayout);
            app.CenterPanel.Layout.Row = 1;
            app.CenterPanel.Layout.Column = 2;

            % Create GridLayout3
            app.GridLayout3 = uigridlayout(app.CenterPanel);
            app.GridLayout3.ColumnWidth = {'1x'};
            app.GridLayout3.RowHeight = {'2x', '1x'};
            app.GridLayout3.Padding = [5 5 5 5];

            % Create UIAxes
            app.UIAxes = uiaxes(app.GridLayout3);
            title(app.UIAxes, 'Temperature vs. Time')
            xlabel(app.UIAxes, 'Time (sec)')
            ylabel(app.UIAxes, 'Temperature (C)')
            app.UIAxes.XGrid = 'on';
            app.UIAxes.YGrid = 'on';
            app.UIAxes.Layout.Row = 1;
            app.UIAxes.Layout.Column = 1;

            % Create UIAxes2
            app.UIAxes2 = uiaxes(app.GridLayout3);
            title(app.UIAxes2, 'PWM Duty Cycle')
            xlabel(app.UIAxes2, 'Time (sec)')
            ylabel(app.UIAxes2, 'Duty Cycle')
            app.UIAxes2.XGrid = 'on';
            app.UIAxes2.YGrid = 'on';
            app.UIAxes2.Layout.Row = 2;
            app.UIAxes2.Layout.Column = 1;

            % Create RightPanel
            app.RightPanel = uipanel(app.GridLayout);
            app.RightPanel.Layout.Row = 1;
            app.RightPanel.Layout.Column = 3;

            % Create GridLayout4
            app.GridLayout4 = uigridlayout(app.RightPanel);
            app.GridLayout4.ColumnWidth = {'2x', '1x'};
            app.GridLayout4.RowHeight = {'1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x'};

            % Create ElapsedTimesecEditFieldLabel
            app.ElapsedTimesecEditFieldLabel = uilabel(app.GridLayout4);
            app.ElapsedTimesecEditFieldLabel.HorizontalAlignment = 'right';
            app.ElapsedTimesecEditFieldLabel.Layout.Row = 1;
            app.ElapsedTimesecEditFieldLabel.Layout.Column = 1;
            app.ElapsedTimesecEditFieldLabel.Text = 'Elapsed Time (sec)';

            % Create ElapsedTimesecEditField
            app.ElapsedTimesecEditField = uieditfield(app.GridLayout4, 'numeric');
            app.ElapsedTimesecEditField.ValueDisplayFormat = '%.3f';
            app.ElapsedTimesecEditField.Editable = 'off';
            app.ElapsedTimesecEditField.Layout.Row = 1;
            app.ElapsedTimesecEditField.Layout.Column = 2;

            % Create TargetTempCEditFieldLabel
            app.TargetTempCEditFieldLabel = uilabel(app.GridLayout4);
            app.TargetTempCEditFieldLabel.HorizontalAlignment = 'right';
            app.TargetTempCEditFieldLabel.Layout.Row = 2;
            app.TargetTempCEditFieldLabel.Layout.Column = 1;
            app.TargetTempCEditFieldLabel.Text = 'Target Temp (C)';

            % Create TargetTempCEditField
            app.TargetTempCEditField = uieditfield(app.GridLayout4, 'numeric');
            app.TargetTempCEditField.ValueDisplayFormat = '%.3f';
            app.TargetTempCEditField.Editable = 'off';
            app.TargetTempCEditField.Layout.Row = 2;
            app.TargetTempCEditField.Layout.Column = 2;

            % Create ReferenceSampleLiveDataPanel
            app.ReferenceSampleLiveDataPanel = uipanel(app.GridLayout4);
            app.ReferenceSampleLiveDataPanel.Title = 'Reference Sample Live Data';
            app.ReferenceSampleLiveDataPanel.Layout.Row = [4 7];
            app.ReferenceSampleLiveDataPanel.Layout.Column = [1 2];

            % Create GridLayout5
            app.GridLayout5 = uigridlayout(app.ReferenceSampleLiveDataPanel);
            app.GridLayout5.ColumnWidth = {'2x', '1x'};
            app.GridLayout5.RowHeight = {'1x', '1x', '1x'};

            % Create TemperatureCEditFieldLabel
            app.TemperatureCEditFieldLabel = uilabel(app.GridLayout5);
            app.TemperatureCEditFieldLabel.HorizontalAlignment = 'right';
            app.TemperatureCEditFieldLabel.Layout.Row = 1;
            app.TemperatureCEditFieldLabel.Layout.Column = 1;
            app.TemperatureCEditFieldLabel.Text = 'Temperature (C)';

            % Create TemperatureCEditField
            app.TemperatureCEditField = uieditfield(app.GridLayout5, 'numeric');
            app.TemperatureCEditField.ValueDisplayFormat = '%.3f';
            app.TemperatureCEditField.Editable = 'off';
            app.TemperatureCEditField.Layout.Row = 1;
            app.TemperatureCEditField.Layout.Column = 2;

            % Create CurrentmAEditFieldLabel
            app.CurrentmAEditFieldLabel = uilabel(app.GridLayout5);
            app.CurrentmAEditFieldLabel.HorizontalAlignment = 'right';
            app.CurrentmAEditFieldLabel.Layout.Row = 2;
            app.CurrentmAEditFieldLabel.Layout.Column = 1;
            app.CurrentmAEditFieldLabel.Text = 'Current (mA)';

            % Create CurrentmAEditField
            app.CurrentmAEditField = uieditfield(app.GridLayout5, 'numeric');
            app.CurrentmAEditField.ValueDisplayFormat = '%.2f';
            app.CurrentmAEditField.Editable = 'off';
            app.CurrentmAEditField.Layout.Row = 2;
            app.CurrentmAEditField.Layout.Column = 2;

            % Create PWMDutyCycleEditFieldLabel
            app.PWMDutyCycleEditFieldLabel = uilabel(app.GridLayout5);
            app.PWMDutyCycleEditFieldLabel.HorizontalAlignment = 'right';
            app.PWMDutyCycleEditFieldLabel.Layout.Row = 3;
            app.PWMDutyCycleEditFieldLabel.Layout.Column = 1;
            app.PWMDutyCycleEditFieldLabel.Text = 'PWM Duty Cycle';

            % Create PWMDutyCycleEditField
            app.PWMDutyCycleEditField = uieditfield(app.GridLayout5, 'numeric');
            app.PWMDutyCycleEditField.ValueDisplayFormat = '%.2f';
            app.PWMDutyCycleEditField.Editable = 'off';
            app.PWMDutyCycleEditField.Layout.Row = 3;
            app.PWMDutyCycleEditField.Layout.Column = 2;

            % Create TestSampleLiveDataPanel
            app.TestSampleLiveDataPanel = uipanel(app.GridLayout4);
            app.TestSampleLiveDataPanel.Title = 'Test Sample Live Data';
            app.TestSampleLiveDataPanel.Layout.Row = [9 12];
            app.TestSampleLiveDataPanel.Layout.Column = [1 2];

            % Create GridLayout6
            app.GridLayout6 = uigridlayout(app.TestSampleLiveDataPanel);
            app.GridLayout6.ColumnWidth = {'2x', '1x'};
            app.GridLayout6.RowHeight = {'1x', '1x', '1x'};

            % Create TemperatureCEditField_2Label
            app.TemperatureCEditField_2Label = uilabel(app.GridLayout6);
            app.TemperatureCEditField_2Label.HorizontalAlignment = 'right';
            app.TemperatureCEditField_2Label.Layout.Row = 1;
            app.TemperatureCEditField_2Label.Layout.Column = 1;
            app.TemperatureCEditField_2Label.Text = 'Temperature (C)';

            % Create TemperatureCEditField_2
            app.TemperatureCEditField_2 = uieditfield(app.GridLayout6, 'numeric');
            app.TemperatureCEditField_2.ValueDisplayFormat = '%.3f';
            app.TemperatureCEditField_2.Editable = 'off';
            app.TemperatureCEditField_2.Layout.Row = 1;
            app.TemperatureCEditField_2.Layout.Column = 2;

            % Create CurrentmAEditField_2Label
            app.CurrentmAEditField_2Label = uilabel(app.GridLayout6);
            app.CurrentmAEditField_2Label.HorizontalAlignment = 'right';
            app.CurrentmAEditField_2Label.Layout.Row = 2;
            app.CurrentmAEditField_2Label.Layout.Column = 1;
            app.CurrentmAEditField_2Label.Text = 'Current (mA)';

            % Create CurrentmAEditField_2
            app.CurrentmAEditField_2 = uieditfield(app.GridLayout6, 'numeric');
            app.CurrentmAEditField_2.ValueDisplayFormat = '%.2f';
            app.CurrentmAEditField_2.Editable = 'off';
            app.CurrentmAEditField_2.Layout.Row = 2;
            app.CurrentmAEditField_2.Layout.Column = 2;

            % Create PWMDutyCycleEditField_2Label
            app.PWMDutyCycleEditField_2Label = uilabel(app.GridLayout6);
            app.PWMDutyCycleEditField_2Label.HorizontalAlignment = 'right';
            app.PWMDutyCycleEditField_2Label.Layout.Row = 3;
            app.PWMDutyCycleEditField_2Label.Layout.Column = 1;
            app.PWMDutyCycleEditField_2Label.Text = 'PWM Duty Cycle';

            % Create PWMDutyCycleEditField_2
            app.PWMDutyCycleEditField_2 = uieditfield(app.GridLayout6, 'numeric');
            app.PWMDutyCycleEditField_2.ValueDisplayFormat = '%.2f';
            app.PWMDutyCycleEditField_2.Editable = 'off';
            app.PWMDutyCycleEditField_2.Layout.Row = 3;
            app.PWMDutyCycleEditField_2.Layout.Column = 2;

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = DSC_PID_Tuning_UI_exported

            runningApp = getRunningApp(app);

            % Check for running singleton app
            if isempty(runningApp)

                % Create UIFigure and components
                createComponents(app)

                % Register the app with App Designer
                registerApp(app, app.UIFigure)

                % Execute the startup function
                runStartupFcn(app, @startupFcn)
            else

                % Focus the running singleton app
                figure(runningApp.UIFigure)

                app = runningApp;
            end

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end