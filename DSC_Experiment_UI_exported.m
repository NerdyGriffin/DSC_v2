classdef DSC_Experiment_UI_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                      matlab.ui.Figure
        GridLayout                    matlab.ui.container.GridLayout
        LeftPanel                     matlab.ui.container.Panel
        GridLayout2                   matlab.ui.container.GridLayout
        SerialPortEditField           matlab.ui.control.EditField
        SerialPortEditFieldLabel      matlab.ui.control.Label
        SetSerialPortButton           matlab.ui.control.Button
        ExperimentParametersPanel     matlab.ui.container.Panel
        GridLayout7                   matlab.ui.container.GridLayout
        HoldTimesecEditField          matlab.ui.control.NumericEditField
        HoldTimesecEditFieldLabel     matlab.ui.control.Label
        RateCminEditField             matlab.ui.control.NumericEditField
        RateCminEditFieldLabel        matlab.ui.control.Label
        EndTempCEditField             matlab.ui.control.NumericEditField
        EndTempCEditFieldLabel        matlab.ui.control.Label
        StartTempCEditField           matlab.ui.control.NumericEditField
        StartTempCEditFieldLabel      matlab.ui.control.Label
        StopExperimentButton          matlab.ui.control.Button
        ApplyExperimentParametersButton  matlab.ui.control.Button
        StartExperimentButton         matlab.ui.control.Button
        LoadConfigFileButton          matlab.ui.control.Button
        CenterPanel                   matlab.ui.container.Panel
        GridLayout3                   matlab.ui.container.GridLayout
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

        TargetLine % Animate line object for the target temperature
        RefSampleLine % Animated line object for the reference sample
        TestSampleLine % Animated line object for the test sample

        % The number of new samples to wait before attempting to update
        % the numeric fields
        DataRefreshDelay = 1;

        % The number of new samples to wait before attempting to refresh
        % the plots
        PlotRefreshDelay = 10;

        Kp
        Ki
        Kd
    end

    methods (Access = public)

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
                    % Create fully-formed filename as a string
                    configFullPath = fullfile(configFilePath, configFileName);

                    % Read the .ini file
                    ini.ReadFile(configFullPath)

                    PIDSection = 'PID Settings';
                    if ini.IsSections(PIDSection)
                        if ini.IsKeys(PIDSection,'Kp')
                            app.Kp = ...
                                ini.GetValues('PID Settings','Kp');
                        end

                        if ini.IsKeys(PIDSection,'Ki')
                            app.Ki = ...
                                ini.GetValues('PID Settings','Ki');
                        end

                        if ini.IsKeys(PIDSection,'Kd')
                            app.Kd = ...
                                ini.GetValues('PID Settings','Kd');
                        end

                    else
                        warningMessage = sprintf("The selected .ini file does not contain a [%s] section", PIDSection);
                        warndlg(warningMessage)
                        warning("The selected .ini file does not contain a [%s] section", PIDSection)
                        configLoadStatus = false;
                        return
                    end

                    TempControlSection = 'Temperature Control';
                    if ini.IsSections(TempControlSection)
                        if ini.IsKeys(TempControlSection,'startTemp')
                            app.StartTempCEditField.Value = ...
                                ini.GetValues(TempControlSection,'startTemp');
                        end

                        if ini.IsKeys(TempControlSection,'endTemp')
                            app.EndTempCEditField.Value = ...
                                ini.GetValues(TempControlSection,'endTemp');
                        end

                        if ini.IsKeys(TempControlSection,'rampUpRate')
                            app.RateCminEditField.Value = ...
                                ini.GetValues(TempControlSection,'rampUpRate');
                        end

                        if ini.IsKeys(TempControlSection,'holdTime')
                            app.HoldTimesecEditField.Value = ...
                                ini.GetValues(TempControlSection,'holdTime');
                        end

                    else
                        warningMessage = sprintf("The selected .ini file does not contain a [%s] section", TempControlSection);
                        warndlg(warningMessage)
                        warning("The selected .ini file does not contain a [%s] section", TempControlSection)
                        configLoadStatus = false;
                        return
                    end
            end
        end

        function sendPIDGains(app)
            % Send the PID gain constants via the serial bus
            flush(app.Arduino);
            write(app.Arduino, 'p', 'char');

            write(app.Arduino, string(app.Kp), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.Ki), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.Kd), 'string');
        end

        function receivePIDGains(app)
            % Receive the PID gain constants via the serial bus
            awaitResponse = true;
            while awaitResponse
                serialData = strip(readline(app.Arduino));
                if strlength(serialData) == 1
                    switch strip(serialData)
                        case 'k'
                            readline(app.Arduino);
                            serialData = strip(readline(app.Arduino));
                            [parsedData, dataIsNum] = str2num(serialData);
                            if dataIsNum && length(parsedData) == 3
                                app.Kp = parsedData(1); %double(readline(app.Arduino));
                                app.Ki = parsedData(2); %double(readline(app.Arduino));
                                app.Kd = parsedData(3); %double(readline(app.Arduino));
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
            flush(app.Arduino);
            write(app.Arduino, 'l', 'char');

            write(app.Arduino, string(app.StartTempCEditField.Value), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.EndTempCEditField.Value), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.RateCminEditField.Value), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.HoldTimesecEditField.Value), 'string');
        end

        function receiveControlParameters(app)
            awaitResponse = true;
            while awaitResponse
                serialData = strip(readline(app.Arduino));
                if strlength(serialData) == 1
                    switch strip(serialData)
                        case 'c'
                            readline(app.Arduino);
                            serialData = strip(readline(app.Arduino));
                            [parsedData, dataIsNum] = str2num(serialData);
                            if dataIsNum && length(parsedData) == 4
                                app.StartTempCEditField.Value = parsedData(1); %double(readline(app.Arduino));
                                app.EndTempCEditField.Value = parsedData(2); %double(readline(app.Arduino));
                                app.RateCminEditField.Value = parsedData(3); %double(readline(app.Arduino));
                                app.HoldTimesecEditField.Value = parsedData(4); %double(readline(app.Arduino));
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
            startDateTime = datetime;
            if not(isfolder('autosave'))
                mkdir('autosave');
            end
            matfileName = ['autosave/autoSaveData-',datestr(startDateTime, 'yyyy-mm-dd-HHMM'),'.mat'];

            elapsedTime = zeros(1,app.PlotRefreshDelay);
            targetTemp = zeros(1,app.PlotRefreshDelay);
            refTemp = zeros(1,app.PlotRefreshDelay);
            sampTemp = zeros(1,app.PlotRefreshDelay);
            refCurrent = zeros(1,app.PlotRefreshDelay);
            sampCurrent = zeros(1,app.PlotRefreshDelay);
            refHeatFlow = zeros(1,app.PlotRefreshDelay);
            sampHeatFlow = zeros(1,app.PlotRefreshDelay);
            refDutyCycle = zeros(1,app.PlotRefreshDelay);
            sampDutyCycle = zeros(1,app.PlotRefreshDelay);

            dataLength = 0;

            experimentIsRunning = true;
            while experimentIsRunning
                serialData = strip(readline(app.Arduino));
                if strlength(serialData) == 1
                    switch strip(serialData)
                        case 'x'
                            experimentIsRunning = false;
                            disp('Received end signal')
                        otherwise
                            disp('Unrecognized data flag while awaiting data:')
                            disp(serialData)
                    end
                else
                    [parsedData, dataIsNum] = str2num(serialData);
                    if dataIsNum && length(parsedData) == 10
                        dataLength = dataLength + 1;
                        elapsedTime(dataLength) = parsedData(1); %str2double(parsedData{1});
                        targetTemp(dataLength) = parsedData(2); %str2double(parsedData{2});
                        refTemp(dataLength) = parsedData(3); %str2double(parsedData{3});
                        sampTemp(dataLength) = parsedData(4); %str2double(parsedData{4});
                        refCurrent(dataLength) = parsedData(5); %str2double(parsedData{5});
                        sampCurrent(dataLength) = parsedData(6); %str2double(parsedData{6});
                        refHeatFlow(dataLength) = parsedData(7); %str2double(parsedData{7});
                        sampHeatFlow(dataLength) = parsedData(8); %str2double(parsedData{8});
                        refDutyCycle(dataLength) = parsedData(9); %str2double(parsedData{9});
                        sampDutyCycle(dataLength) = parsedData(10); %str2double(parsedData{10});

                        if ~mod(dataLength, app.DataRefreshDelay)
                            updateLiveData(app, ...
                                elapsedTime(dataLength), ...
                                targetTemp(dataLength), ...
                                refTemp(dataLength), ...
                                sampTemp(dataLength), ...
                                refCurrent(dataLength), ...
                                sampCurrent(dataLength), ...
                                refDutyCycle(dataLength), ...
                                sampDutyCycle(dataLength));
                        end

                        if ~mod(dataLength, app.PlotRefreshDelay)
                            refreshLivePlot(app, elapsedTime,...
                                targetTemp, refTemp, sampTemp);
                        end
                    else
                        disp(parsedData)
                    end
                end
            end

            save(matfileName, 'startDateTime', 'elapsedTime', 'targetTemp', ...
                'refTemp', 'sampTemp', ...
                'refCurrent', 'sampCurrent', ...
                'refHeatFlow', 'sampHeatFlow', ...
                'refDutyCycle', 'sampDutyCycle', 'dataLength')
            if isfile(matfileName)
                fprintf('Autosave file created: %s\n', matfileName)
            end

            updateLiveData(app, ...
                elapsedTime(dataLength), ...
                targetTemp(dataLength), ...
                refTemp(dataLength), ...
                sampTemp(dataLength), ...
                refCurrent(dataLength), ...
                sampCurrent(dataLength), ...
                refDutyCycle(dataLength), ...
                sampDutyCycle(dataLength));

            refreshLivePlot(app, elapsedTime, targetTemp, refTemp, sampTemp);

            setIdleUI(app);
        end

        function setRunningUI(app)
            app.StartExperimentButton.Enable = 'off';
            app.StopExperimentButton.Enable = 'on';
            app.LoadConfigFileButton.Enable = 'off';
            app.ApplyExperimentParametersButton.Enable = 'off';
            app.StartTempCEditField.Editable = 'off';
            app.EndTempCEditField.Editable = 'off';
            app.RateCminEditField.Editable = 'off';
            app.HoldTimesecEditField.Editable = 'off';
            app.SetSerialPortButton.Enable = 'off';
            app.SerialPortEditField.Editable = 'off';
        end

        function setIdleUI(app)
            app.StartExperimentButton.Enable = 'on';
            app.StopExperimentButton.Enable = 'off';
            app.LoadConfigFileButton.Enable = 'on';
            app.ApplyExperimentParametersButton.Enable = 'on';
            app.StartTempCEditField.Editable = 'on';
            app.EndTempCEditField.Editable = 'on';
            app.RateCminEditField.Editable = 'on';
            app.HoldTimesecEditField.Editable = 'on';
            app.SetSerialPortButton.Enable = 'on';
            app.SerialPortEditField.Editable = 'on';
        end

        function updateLiveData(app, elapsedTime, targetTemp, ...
                refTemp, sampTemp, refCurrent, sampCurrent, ...
                refDutyCycle, sampDutyCycle)
            drawnow limitrate nocallbacks

            % Convert from milliseconds to seconds
            app.ElapsedTimesecEditField.Value = elapsedTime / 1000;

            app.TargetTempCEditField.Value = targetTemp;

            app.TemperatureCEditField.Value = refTemp;
            app.CurrentmAEditField.Value = refCurrent;
            app.PWMDutyCycleEditField.Value = refDutyCycle;

            app.TemperatureCEditField_2.Value = sampTemp;
            app.CurrentmAEditField_2.Value = sampCurrent;
            app.PWMDutyCycleEditField_2.Value = sampDutyCycle;

            drawnow limitrate
        end

        function refreshLivePlot(app, elapsedTimeArray,...
                targetTempArray, refTempArray, sampTempArray)

            % Convert from milliseconds to seconds
            timeInSeconds = elapsedTimeArray ./ 1000;

            drawnow limitrate nocallbacks

            clearpoints(app.TargetLine)
            clearpoints(app.RefSampleLine)
            clearpoints(app.TestSampleLine)

            % Update the plots
            addpoints(app.TargetLine, timeInSeconds, targetTempArray)
            addpoints(app.RefSampleLine, timeInSeconds, refTempArray)
            addpoints(app.TestSampleLine, timeInSeconds, sampTempArray)

            legend(app.UIAxes, 'Location', 'best')

            drawnow limitrate
        end
    end


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            % Get the list of available serial ports
            app.SerialPortList = serialportlist("available");

            if (isempty(app.SerialPortList))
                % Display a warning if no serial ports found
                warndlg('No available serial ports were found. Make sure the arduino device is plugged into this computer via USB')
            else
                % Set the default serial port to the last port in the list
                app.SerialPort = app.SerialPortList(end);

                % Update the serial port edit field
                app.SerialPortEditField.Value = app.SerialPort;

                % Create an serial port object where you specify the USB port
                % (look in Arduino->tools -> port and the baud rate (9600)
                app.Arduino = serialport(app.SerialPort, 9600);

                % Request the temperature control parameters from the arduino
                flush(app.Arduino);
                write(app.Arduino, 'i', 'char');
                receivePIDGains(app);
                receiveControlParameters(app);
            end

            % Create the animatedline objects
            app.TargetLine = animatedline(app.UIAxes, 'Color', 'black', ...
                'LineStyle', ':');
            app.RefSampleLine = animatedline(app.UIAxes, 'Color', 'blue', ...
                'LineStyle', '--');
            app.TestSampleLine = animatedline(app.UIAxes, 'Color', 'red', ...
                'LineStyle', '-.');

            % Create a legend for the temperature plot
            legend(app.UIAxes, 'Target Temperature', 'Reference Sample', ...
                'Test Sample', 'Location', 'best')

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

        % Button pushed function: StopExperimentButton
        function StopExperimentButtonPushed(app, event)
            app.StopExperimentButton.Enable = 'off';

            write(app.Arduino, 'x', 'char');
        end

        % Button pushed function: LoadConfigFileButton
        function LoadConfigFileButtonPushed(app, event)
            app.LoadConfigFileButton.Enable = 'off';

            loadConfigFile(app);

            sendPIDGains(app);
            receivePIDGains(app);

            sendControlParameters(app);
            receiveControlParameters(app);

            app.LoadConfigFileButton.Enable = 'on';
        end

        % Button pushed function: ApplyExperimentParametersButton
        function ApplyExperimentParametersButtonPushed(app, event)
            app.ApplyExperimentParametersButton.Enable = 'off';

            sendPIDGains(app);
            receivePIDGains(app);

            sendControlParameters(app);
            receiveControlParameters(app);

            app.ApplyExperimentParametersButton.Enable = 'on';
        end

        % Button pushed function: SetSerialPortButton
        function SetSerialPortButtonPushed(app, event)
            app.SetSerialPortButton.Enable = 'off';

            delete(app.Arduino);

            % Get the list of available serial ports
            app.SerialPortList = serialportlist("available");

            if any(contains(app.SerialPortList, app.SerialPortEditField.Value))
                app.SerialPort = app.SerialPortEditField.Value;

                % Create an serial port object where you specify the USB port
                % (look in Arduino->tools -> port and the baud rate (9600)
                app.Arduino = serialport(app.SerialPort, 9600);

                % Request the temperature control parameters from the arduino
                write(app.Arduino, 'i', 'char');
                receivePIDGains(app);
                receiveControlParameters(app);
            else
                warndlg(sprintf("%s is not in the list of available serial ports", app.SerialPortEditField.Value));
                disp(app.SerialPortList)
                disp(app.SerialPortEditField.Value)
            end

            app.SetSerialPortButton.Enable = 'on';
        end

        % Close request function: UIFigure
        function UIFigureCloseRequest(app, event)
            delete(app)
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

            % Create ApplyExperimentParametersButton
            app.ApplyExperimentParametersButton = uibutton(app.GridLayout2, 'push');
            app.ApplyExperimentParametersButton.ButtonPushedFcn = createCallbackFcn(app, @ApplyExperimentParametersButtonPushed, true);
            app.ApplyExperimentParametersButton.BackgroundColor = [0 1 1];
            app.ApplyExperimentParametersButton.Layout.Row = 4;
            app.ApplyExperimentParametersButton.Layout.Column = [1 2];
            app.ApplyExperimentParametersButton.Text = 'Apply Experiment Parameters';

            % Create StopExperimentButton
            app.StopExperimentButton = uibutton(app.GridLayout2, 'push');
            app.StopExperimentButton.ButtonPushedFcn = createCallbackFcn(app, @StopExperimentButtonPushed, true);
            app.StopExperimentButton.BackgroundColor = [1 0 0];
            app.StopExperimentButton.Layout.Row = 2;
            app.StopExperimentButton.Layout.Column = [1 2];
            app.StopExperimentButton.Text = 'Stop Experiment';

            % Create ExperimentParametersPanel
            app.ExperimentParametersPanel = uipanel(app.GridLayout2);
            app.ExperimentParametersPanel.Title = 'Experiment Parameters';
            app.ExperimentParametersPanel.Layout.Row = [5 8];
            app.ExperimentParametersPanel.Layout.Column = [1 2];

            % Create GridLayout7
            app.GridLayout7 = uigridlayout(app.ExperimentParametersPanel);
            app.GridLayout7.ColumnWidth = {'2x', '1x'};
            app.GridLayout7.RowHeight = {'1x', '1x', '1x', '1x'};

            % Create StartTempCEditFieldLabel
            app.StartTempCEditFieldLabel = uilabel(app.GridLayout7);
            app.StartTempCEditFieldLabel.HorizontalAlignment = 'right';
            app.StartTempCEditFieldLabel.Layout.Row = 1;
            app.StartTempCEditFieldLabel.Layout.Column = 1;
            app.StartTempCEditFieldLabel.Text = 'Start Temp (C)';

            % Create StartTempCEditField
            app.StartTempCEditField = uieditfield(app.GridLayout7, 'numeric');
            app.StartTempCEditField.Limits = [-300 300];
            app.StartTempCEditField.Layout.Row = 1;
            app.StartTempCEditField.Layout.Column = 2;

            % Create EndTempCEditFieldLabel
            app.EndTempCEditFieldLabel = uilabel(app.GridLayout7);
            app.EndTempCEditFieldLabel.HorizontalAlignment = 'right';
            app.EndTempCEditFieldLabel.Layout.Row = 2;
            app.EndTempCEditFieldLabel.Layout.Column = 1;
            app.EndTempCEditFieldLabel.Text = 'End Temp (C)';

            % Create EndTempCEditField
            app.EndTempCEditField = uieditfield(app.GridLayout7, 'numeric');
            app.EndTempCEditField.Limits = [-300 300];
            app.EndTempCEditField.Layout.Row = 2;
            app.EndTempCEditField.Layout.Column = 2;

            % Create RateCminEditFieldLabel
            app.RateCminEditFieldLabel = uilabel(app.GridLayout7);
            app.RateCminEditFieldLabel.HorizontalAlignment = 'right';
            app.RateCminEditFieldLabel.Layout.Row = 3;
            app.RateCminEditFieldLabel.Layout.Column = 1;
            app.RateCminEditFieldLabel.Text = 'Rate (C/min)';

            % Create RateCminEditField
            app.RateCminEditField = uieditfield(app.GridLayout7, 'numeric');
            app.RateCminEditField.Limits = [0 Inf];
            app.RateCminEditField.Layout.Row = 3;
            app.RateCminEditField.Layout.Column = 2;

            % Create HoldTimesecEditFieldLabel
            app.HoldTimesecEditFieldLabel = uilabel(app.GridLayout7);
            app.HoldTimesecEditFieldLabel.HorizontalAlignment = 'right';
            app.HoldTimesecEditFieldLabel.Layout.Row = 4;
            app.HoldTimesecEditFieldLabel.Layout.Column = 1;
            app.HoldTimesecEditFieldLabel.Text = 'Hold Time (sec)';

            % Create HoldTimesecEditField
            app.HoldTimesecEditField = uieditfield(app.GridLayout7, 'numeric');
            app.HoldTimesecEditField.Limits = [0 Inf];
            app.HoldTimesecEditField.Layout.Row = 4;
            app.HoldTimesecEditField.Layout.Column = 2;

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
            app.SerialPortEditField.Value = 'COM1';

            % Create CenterPanel
            app.CenterPanel = uipanel(app.GridLayout);
            app.CenterPanel.Layout.Row = 1;
            app.CenterPanel.Layout.Column = 2;

            % Create GridLayout3
            app.GridLayout3 = uigridlayout(app.CenterPanel);
            app.GridLayout3.ColumnWidth = {'1x'};
            app.GridLayout3.RowHeight = {'1x'};
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
        function app = DSC_Experiment_UI_exported

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