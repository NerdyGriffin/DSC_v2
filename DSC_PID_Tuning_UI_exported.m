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

    properties (Access = private)
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

        StartTemp
        EndTemp
        RampUpRate
        HoldTime
    end

    methods (Access = private)

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
                            app.KpEditField.Value = ...
                                ini.GetValues('PID Constants','Kp');
                        end

                        if ini.IsKeys(PIDSection,'Ki')
                            app.KiEditField.Value = ...
                                ini.GetValues('PID Constants','Ki');
                        end

                        if ini.IsKeys(PIDSection,'Kd')
                            app.KdEditField.Value = ...
                                ini.GetValues('PID Constants','Kd');
                        end

                    else
                        warningMessage = sprintf("The seleced .ini file does not contain a [%s] section", PIDSection);
                        warndlg(warningMessage)
                        warning(warningMessage)
                        configLoadStatus = false;
                        return
                    end

                    TempControlSection = 'Temperature Control';
                    if ini.IsSections(TempControlSection)
                        if ini.IsKeys(TempControlSection,'startTemp')
                            app.StartTemp = ...
                                ini.GetValues(TempControlSection,'startTemp');
                        end

                        if ini.IsKeys(TempControlSection,'endTemp')
                            app.EndTemp = ...
                                ini.GetValues(TempControlSection,'endTemp');
                        end

                        if ini.IsKeys(TempControlSection,'rampUpRate')
                            app.RampUpRate = ...
                                ini.GetValues(TempControlSection,'rampUpRate');
                        end

                        if ini.IsKeys(TempControlSection,'holdTime')
                            app.HoldTime = ...
                                ini.GetValues(TempControlSection,'holdTime');
                        end

                    else
                        warningMessage = sprintf("The seleced .ini file does not contain a [%s] section", TempControlSection);
                        warndlg(warningMessage)
                        warning(warningMessage)
                        configLoadStatus = false;
                        return
                    end
            end
        end

        function sendPIDGains(app)
            % Send the PID gain constants via the serial bus
            write(app.Arduino, 'p', 'char');

            write(app.Arduino, string(app.KpEditField.Value), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.KiEditField.Value), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.KdEditField.Value), 'string');
        end

        function receivePIDGains(app)
            % Receive the PID gain constants via the serial bus
            for awaitData = 1:100
                serialData = readline(app.Arduino);
                if length(serialData) == 1 && serialData == 'k'
                    app.KpEditField.Value = double(readline(app.Arduino));
                    app.KiEditField.Value = double(readline(app.Arduino));
                    app.KdEditField.Value = double(readline(app.Arduino));
                    break
                else
                    disp('Unrecognized control param flag:')
                    disp(serialData)
                end
            end
        end

        function sendControlParameters(app)
            write(app.Arduino, 'l', 'char');

            write(app.Arduino, string(app.StartTemp), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.EndTemp), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.RampUpRate), 'string');
            write(app.Arduino, ' ', 'char');
            write(app.Arduino, string(app.HoldTime), 'string');
        end

        function receiveControlParameters(app)
            for awaitData = 1:100
                serialData = readline(app.Arduino);
                if length(serialData) == 1 && serialData == 'c'
                    app.StartTemp = double(readline(app.Arduino));
                    app.EndTemp = double(readline(app.Arduino));
                    app.RampUpRate = double(readline(app.Arduino));
                    app.HoldTime = double(readline(app.Arduino));
                    break
                else
                    disp('Unrecognized control param flag:')
                    disp(serialData)
                end
            end
        end

        function receiveSerialData(app)
            startDateTime = datetime;
            mkdir('autosave');
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
                serialData = readline(app.Arduino);
                if length(serialData) == 1
                    switch serialData
                        case 'x'
                            experimentIsRunning = false;
                            disp('Received end signal')
                        otherwise
                            disp('Unrecognized control param flag:')
                            disp(serialData)
                    end
                else
                    parsedData = strsplit(serialData, ',');
                    if length(parsedData) == 10
                        dataLength = dataLength + 1;
                        elapsedTime(dataLength) = str2double(parsedData{1});
                        targetTemp(dataLength) = str2double(parsedData{2});
                        refTemp(dataLength) = str2double(parsedData{3});
                        sampTemp(dataLength) = str2double(parsedData{4});
                        refCurrent(dataLength) = str2double(parsedData{5});
                        sampCurrent(dataLength) = str2double(parsedData{6});
                        refHeatFlow(dataLength) = str2double(parsedData{7});
                        sampHeatFlow(dataLength) = str2double(parsedData{8});
                        refDutyCycle(dataLength) = str2double(parsedData{9});
                        sampDutyCycle(dataLength) = str2double(parsedData{10});

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
                                targetTemp, refTemp, sampTemp,...
                                refDutyCycle, sampDutyCycle);
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

            updateLiveData(app, ...
                elapsedTime(dataLength), ...
                targetTemp(dataLength), ...
                refTemp(dataLength), ...
                sampTemp(dataLength), ...
                refCurrent(dataLength), ...
                sampCurrent(dataLength), ...
                refDutyCycle(dataLength), ...
                sampDutyCycle(dataLength));

            refreshLivePlot(app, elapsedTime, targetTemp,...
                refTemp, sampTemp,...
                refDutyCycle, sampDutyCycle);

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
                targetTempArray, refTempArray, sampTempArray,...
                refDutyCycleArray, sampDutyCycleArray)

            % Convert from milliseconds to seconds
            timeInSeconds = elapsedTimeArray ./ 1000;

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
            BANG_RANGE = 10;
            MINIMUM_ACCEPTABLE_ERROR = 5;

            % Update the plots
            addpoints(app.BangOffLine, timeInSeconds, targetTempArray + BANG_RANGE)
            addpoints(app.TargetMaxLine, timeInSeconds, targetTempArray + MINIMUM_ACCEPTABLE_ERROR)
            addpoints(app.TargetLine, timeInSeconds, targetTempArray)
            addpoints(app.TargetMinLine, timeInSeconds, targetTempArray - MINIMUM_ACCEPTABLE_ERROR)
            addpoints(app.BangOnLine, timeInSeconds, targetTempArray - BANG_RANGE)
            addpoints(app.RefSampleLine, timeInSeconds, refTempArray)
            addpoints(app.TestSampleLine, timeInSeconds, sampTempArray)

            addpoints(app.RefDutyCycleLine, timeInSeconds, refDutyCycleArray)
            addpoints(app.SampDutyCycleLine, timeInSeconds, sampDutyCycleArray)

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
                write(app.Arduino, 'i', 'char');
                receivePIDGains(app);
                receiveControlParameters(app);
            end

            % Create the animatedline objects
            app.BangOffLine = animatedline(app.UIAxes, 'Color', 'yellow', ...
                'LineStyle', ':');
            app.TargetMaxLine = animatedline(app.UIAxes, 'Color', 'green', ...
                'LineStyle', ':');
            app.TargetLine = animatedline(app.UIAxes, 'Color', 'black', ...
                'LineStyle', ':');
            app.TargetMinLine = animatedline(app.UIAxes, 'Color', 'green', ...
                'LineStyle', ':');
            app.BangOnLine = animatedline(app.UIAxes, 'Color', 'yellow', ...
                'LineStyle', ':');
            app.RefSampleLine = animatedline(app.UIAxes, 'Color', 'blue', ...
                'LineStyle', '--');
            app.TestSampleLine = animatedline(app.UIAxes, 'Color', 'red', ...
                'LineStyle', '-.');

            app.RefDutyCycleLine = animatedline(app.UIAxes2, 'Color', 'blue', ...
                'LineStyle', '--');
            app.SampDutyCycleLine = animatedline(app.UIAxes2, 'Color', 'red', ...
                'LineStyle', '-.');

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

            write(app.Arduino, 's', 'char');

            for i = 1:100
                serialData = readline(app.Arduino);
                if length(serialData) == 1 && serialData == 's'
                    setRunningUI(app);
                    receiveSerialData(app);
                    break
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

        % Button pushed function: ApplyPIDParametersButton
        function ApplyPIDParametersButtonPushed(app, event)
            app.ApplyPIDParametersButton.Enable = 'off';

            sendPIDGains(app);
            receivePIDGains(app);

            sendControlParameters(app);
            receiveControlParameters(app);

            app.ApplyPIDParametersButton.Enable = 'on';
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
            app.GridLayout7.ColumnWidth = {'2x', '1x'};
            app.GridLayout7.RowHeight = {'1x', '1x', '1x', '1x'};

            % Create KpEditFieldLabel
            app.KpEditFieldLabel = uilabel(app.GridLayout7);
            app.KpEditFieldLabel.HorizontalAlignment = 'right';
            app.KpEditFieldLabel.Layout.Row = 1;
            app.KpEditFieldLabel.Layout.Column = 1;
            app.KpEditFieldLabel.Text = 'Kp';

            % Create KpEditField
            app.KpEditField = uieditfield(app.GridLayout7, 'numeric');
            app.KpEditField.Layout.Row = 1;
            app.KpEditField.Layout.Column = 2;

            % Create KiEditFieldLabel
            app.KiEditFieldLabel = uilabel(app.GridLayout7);
            app.KiEditFieldLabel.HorizontalAlignment = 'right';
            app.KiEditFieldLabel.Layout.Row = 2;
            app.KiEditFieldLabel.Layout.Column = 1;
            app.KiEditFieldLabel.Text = 'Ki';

            % Create KiEditField
            app.KiEditField = uieditfield(app.GridLayout7, 'numeric');
            app.KiEditField.Layout.Row = 2;
            app.KiEditField.Layout.Column = 2;

            % Create KdEditFieldLabel
            app.KdEditFieldLabel = uilabel(app.GridLayout7);
            app.KdEditFieldLabel.HorizontalAlignment = 'right';
            app.KdEditFieldLabel.Layout.Row = 3;
            app.KdEditFieldLabel.Layout.Column = 1;
            app.KdEditFieldLabel.Text = 'Kd';

            % Create KdEditField
            app.KdEditField = uieditfield(app.GridLayout7, 'numeric');
            app.KdEditField.Layout.Row = 3;
            app.KdEditField.Layout.Column = 2;

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
            app.SerialPortEditField.Value = 'COM3';

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