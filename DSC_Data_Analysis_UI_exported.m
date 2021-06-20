classdef DSC_Data_Analysis_UI_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure               matlab.ui.Figure
        GridLayout             matlab.ui.container.GridLayout
        LeftPanel              matlab.ui.container.Panel
        GridLayout2            matlab.ui.container.GridLayout
        YAxisDataPanel         matlab.ui.container.Panel
        GridLayout7            matlab.ui.container.GridLayout
        TemperatureCButton_Y   matlab.ui.control.StateButton
        HeatFlowRateWgButton   matlab.ui.control.StateButton
        XAxisDataPanel         matlab.ui.container.Panel
        GridLayout6            matlab.ui.container.GridLayout
        TemperatureCButton_X   matlab.ui.control.StateButton
        TimesecButton          matlab.ui.control.StateButton
        DataSetSelectionPanel  matlab.ui.container.Panel
        GridLayout5            matlab.ui.container.GridLayout
        TestSampButton         matlab.ui.control.StateButton
        ReferenceButton        matlab.ui.control.StateButton
        TargetTempButton       matlab.ui.control.StateButton
        DifferentialButton     matlab.ui.control.StateButton
        LoadMATfileButton      matlab.ui.control.Button
        RightPanel             matlab.ui.container.Panel
        GridLayout3            matlab.ui.container.GridLayout
        UIAxes                 matlab.ui.control.UIAxes
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
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
        MatObj % Description
    end
    
    methods (Access = private)
        
        function dataLoadStatus = loadMATFile(app)
            %   Load data from a .mat file for analysis
            
            % Prompt the user to select a file
            [dataFileName, dataFilePath] = uigetfile('*.mat');
            
            switch dataFileName
                case 0
                    % Cancel the read operation and return an empty array
                    % if the user closes the file selection window
                    dataLoadStatus = false;
                    return
                    
                otherwise
                    % Create fully-formed filename as a string
                    dataFullPath = fullfile(dataFilePath, dataFileName);
                    
                    % Load the .mat files into a MatFile object
                    temporaryMatObj = matfile(dataFullPath,'Writable',false);
                    
                    varlist = who(temporaryMatObj);
                    
                    expectedVarlist = ...
                        {'dataLength'; ...
                        'elapsedTime'; ...
                        'refCurrent'; ...
                        'refDutyCycle'; ...
                        'refHeatFlow'; ...
                        'refTemp'; ...
                        'sampCurrent'; ...
                        'sampDutyCycle'; ...
                        'sampHeatFlow'; ...
                        'sampTemp'; ...
                        'startDateTime'; ...
                        'targetTemp'};
                    
                    if contains(varlist, expectedVarlist)
                        app.MatObj = temporaryMatObj;
                        dataLoadStatus = true;
                    else
                        warningMessage = 'The selected .mat file does not contain the expected variables';
                        warndlg(warningMessage)
                        warning(warningMessage)
                        dataLoadStatus = false;
                        return
                    end
            end
        end
        
        function updateUIStates(app)
            % Update the states of the data selection buttons
            drawnow nocallbacks
            
            if (app.MatObj.dataLength > 0)
                % Enable the UI once data has been loaded
                app.DifferentialButton.Enable = 'on';
                app.TargetTempButton.Enable = 'on';
                app.ReferenceButton.Enable = 'on';
                app.TestSampButton.Enable = 'on';
                
                app.TimesecButton.Enable = 'on';
                app.TemperatureCButton_X.Enable = 'on';
                
                app.HeatFlowRateWgButton.Enable = 'on';
                app.TemperatureCButton_Y.Enable = 'on';
                
                if app.DifferentialButton.Value
                    app.HeatFlowRateWgButton.Enable = 'on';
                elseif app.TargetTempButton.Value
                    app.HeatFlowRateWgButton.Enable = 'off';
                    app.TimesecButton.Value = true;
                    app.TemperatureCButton_X.Value = false;
                    app.TemperatureCButton_X.Enable = 'off';
                elseif app.ReferenceButton.Value || app.TestSampButton.Value
                    app.HeatFlowRateWgButton.Enable = 'on';
                end
                
                redrawPlot(app)
            end
            
            drawnow
        end
        
        function redrawPlot(app)
            drawnow nocallbacks
            
            % Clear the previous plot so that the new plot may be drawn
            cla(app.UIAxes)
            
            % Turn on "hold" for the plot axes
            hold(app.UIAxes, 'on')
            
            if app.TimesecButton.Value
                app.TemperatureCButton_Y.Enable = 'on';
                
                if app.HeatFlowRateWgButton.Value
                    app.TemperatureCButton_X.Enable = 'on';
                    title(app.UIAxes, 'Heat Flow Rate vs. Time')
                    xlabel(app.UIAxes, 'Time (sec)')
                    ylabel(app.UIAxes, 'Heat Flow Rate (W/g)')
                    
                    % Check whether there is any data before attempting to
                    % plot
                    if (app.MatObj.dataLength > 0)
                        % Calculate the range of the time data
                        [minTime, maxTime] = bounds(app.MatObj.elapsedTime);
                        timeDataRange = [floor(minTime), ceil(maxTime)];
                        
                        if app.DifferentialButton.Value
                            % Calculate the Differential Heat Flow
                            diffHeatFlow = app.MatObj.sampHeatFlow - app.MatObj.refHeatFlow;
                            
                            % Plot the Differential data as Heat Flow Rate
                            % vs. Time
                            line(app.UIAxes, app.MatObj.elapsedTime, diffHeatFlow,...
                                'DisplayName', 'Differential')
                            if ~isequal(0, abs(maxTime - minTime))
                                line(app.UIAxes, timeDataRange, [0, 0],...
                                    'Color', 'black', 'LineStyle', ':', ...
                                    'DisplayName', 'Zero');
                            end
                        end
                        
                        if app.ReferenceButton.Value
                            % Plot the Reference Sample data as Heat Flow
                            % Rate vs. Time
                            line(app.UIAxes, app.MatObj.elapsedTime, app.MatObj.refHeatFlow,...
                                'Color', 'blue', 'LineStyle', '-', ...
                                'DisplayName', 'Reference');
                        end
                        
                        if app.TestSampButton.Value
                            % Plot the Test Sample data as Heat Flow Rate
                            % vs. Time
                            line(app.UIAxes, app.MatObj.elapsedTime, app.MatObj.sampHeatFlow,...
                                'Color', 'red', 'LineStyle', '-', ...
                                'DisplayName', 'Test Sample');
                        end
                    end
                    
                elseif app.TemperatureCButton_Y.Value
                    app.TemperatureCButton_X.Enable = 'off';
                    title(app.UIAxes, 'Temperature vs. Time')
                    xlabel(app.UIAxes, 'Time (sec)')
                    ylabel(app.UIAxes, 'Temperature (\circC)')
                    
                    % Check whether there is any data before attempting to
                    % plot
                    if (app.MatObj.dataLength > 0)
                        % Calculate the range of the time data
                        [minTime, maxTime] = bounds(app.MatObj.elapsedTime);
                        timeDataRange = [floor(minTime), ceil(maxTime)];
                        
                        if app.DifferentialButton.Value
                            % Plot the Differential data as Temperature vs.
                            % Time
                            line(app.UIAxes, app.MatObj.elapsedTime, (app.MatObj.sampTemp - app.MatObj.refTemp),...
                                'DisplayName', 'Differential');
                            if ~isequal(0, abs(maxTime - minTime))
                                line(app.UIAxes, timeDataRange, [0, 0],...
                                    'Color', 'black', 'LineStyle', ':', ...
                                    'DisplayName', 'Zero');
                            end
                        end
                        
                        if app.ReferenceButton.Value
                            % Plot the Reference Sample data as Temperature
                            % vs. Time
                            line(app.UIAxes, app.MatObj.elapsedTime, app.MatObj.refTemp,...
                                'Color', 'blue', 'LineStyle', '-', ...
                                'DisplayName', 'Reference');
                            
                        end
                        
                        if app.TestSampButton.Value
                            % Plot the Test Sample data as Temperature vs.
                            % Time
                            line(app.UIAxes, app.MatObj.elapsedTime, app.MatObj.sampTemp,...
                                'Color', 'red', 'LineStyle', '-', ...
                                'DisplayName', 'Test Sample');
                            
                        end
                        
                        if app.TargetTempButton.Value
                            % Plot the Target Temp data as Temperature vs.
                            % Time
                            line(app.UIAxes, app.MatObj.elapsedTime, app.MatObj.targetTemp,...
                                'Color', 'black', 'LineStyle', ':', ...
                                'DisplayName', 'Target Temp');
                            
                        end
                    end
                end
                
            elseif app.TemperatureCButton_X.Value
                app.TemperatureCButton_Y.Enable = 'off';
                
                app.HeatFlowRateWgButton.Value = true;
                app.TemperatureCButton_Y.Value = false;
                title(app.UIAxes, 'Heat Flow Rate vs. Temperature')
                xlabel(app.UIAxes, 'Temperature (\circC)')
                ylabel(app.UIAxes, 'Heat Flow Rate (W/g)')
                
                % Check whether there is any data before attempting to plot
                if (app.MatObj.dataLength > 0)
                    % Calculate the range of the temperature data
                    tempData_Combined = [app.MatObj.refTemp, app.MatObj.sampTemp];
                    [minTemp, maxTemp] = bounds(tempData_Combined);
                    tempDataRange = [floor(minTemp), ceil(maxTemp)];
                    
                    % Calculate the range of the heat flow rate data
                    [minRefHeatFlow, maxRefHeatFlow] = bounds(app.MatObj.refHeatFlow);
                    [minSampHeatFlow, maxSampHeatFlow] = bounds(app.MatObj.sampHeatFlow);
                    
                    
                    switch app.MatObj.dataLength
                        case 0
                            % This case should never be executed
                        case 1
                            interpolatedTemp = tempDataRange;
                            interpolatedRefHeatFlow = [minRefHeatFlow, maxRefHeatFlow];
                            interpolatedSampHeatFlow = [minSampHeatFlow, maxSampHeatFlow];
                        case 2
                            interpolatedTemp = tempDataRange;
                            interpolatedRefHeatFlow = app.MatObj.refHeatFlow;
                            interpolatedSampHeatFlow = app.MatObj.sampHeatFlow;
                            
                        otherwise
                            % Interpolate the temperature data
                            interpolatedTemp = linspace(minTemp, maxTemp,...
                                2 * app.MatObj.dataLength);
                            
                            % Interpolation requires that sample points
                            % must be unique and sorted in ascending order
                            
                            % Store corresponding temperature and heat flow
                            % data
                            unsortedRefData = [app.MatObj.refTemp', app.MatObj.refHeatFlow'];
                            unsortedSampData = [app.MatObj.sampTemp', app.MatObj.sampHeatFlow'];
                            
                            % Sort the data by temperature
                            sortedRefData = sortrows(unsortedRefData);
                            sortedSampData = sortrows(unsortedSampData);
                            
                            % Find unique x values
                            [~,refIdx] = unique(sortedRefData(:,1));
                            [~,sampIdx] = unique(sortedSampData(:,1));
                            
                            % Remove rows with duplicate x values
                            uniqueRefData = sortedRefData(refIdx,:);
                            uniqueSampData = sortedSampData(sampIdx,:);
                            
                            % Store the sorted data in separate arrays
                            sortedRefTemp = uniqueRefData(:,1);
                            sortedRefHeatFlow = uniqueRefData(:,2);
                            sortedSampTemp = uniqueSampData(:,1);
                            sortedSampHeatFlow = uniqueSampData(:,2);
                            
                            % Interpolate the heat flow data
                            interpolatedRefHeatFlow = ...
                                spline(sortedRefTemp, sortedRefHeatFlow,...
                                interpolatedTemp);
                            interpolatedSampHeatFlow = ...
                                spline(sortedSampTemp, sortedSampHeatFlow,...
                                interpolatedTemp);
                    end
                    
                    
                    interpolatedDiffHeatFlow = ...
                        interpolatedSampHeatFlow - interpolatedRefHeatFlow;
                    
                    
                    if app.DifferentialButton.Value
                        % Plot the Differential data as Heat Flow Rate vs. Temperature
                        line(app.UIAxes, interpolatedTemp, interpolatedDiffHeatFlow,...
                            'DisplayName', 'Differential');
                        if ~isequal(0, abs(maxTemp - minTemp))
                            line(app.UIAxes, tempDataRange, [0, 0],...
                                'Color', 'black', 'LineStyle', ':', ...
                                'DisplayName', 'Zero');
                        end
                    end
                    
                    if app.ReferenceButton.Value
                        % Plot the Reference Sample data as Heat Flow Rate vs. Temperature
                        line(app.UIAxes, app.MatObj.refTemp, app.MatObj.refHeatFlow,...
                            'Color', 'blue', 'LineStyle', '-', ...
                            'DisplayName', 'Reference');
                        
                    end
                    
                    if app.TestSampButton.Value
                        % Plot the Test Sample data as Heat Flow Rate vs. Temperature
                        line(app.UIAxes, app.MatObj.sampTemp, app.MatObj.sampHeatFlow,...
                            'Color', 'red', 'LineStyle', '-', ...
                            'DisplayName', 'Test Sample');
                        
                    end
                end
            end
            
            legend(app.UIAxes, 'Location', 'best')
            
            % Turn off "hold" for the plot
            hold(app.UIAxes, 'off')
            
            drawnow
            
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: LoadMATfileButton
        function LoadMATfileButtonPushed(app, event)
            app.LoadMATfileButton.Enable = 'off';
            
            loadMATFile(app);
            
            updateUIStates(app);
            
            app.LoadMATfileButton.Enable = 'on';
            
        end

        % Value changed function: DifferentialButton
        function DifferentialButtonValueChanged(app, event)
            value = app.DifferentialButton.Value;
            if value
                app.TargetTempButton.Value = false;
                app.ReferenceButton.Value = false;
                app.TestSampButton.Value = false;
                
                app.TimesecButton.Value = false;
                app.TemperatureCButton_X.Value = true;
                
                app.HeatFlowRateWgButton.Value = true;
                app.TemperatureCButton_Y.Value = false;
            end
            
            updateUIStates(app);
            
        end

        % Value changed function: TargetTempButton
        function TargetTempButtonValueChanged(app, event)
            value = app.TargetTempButton.Value;
            if value
                app.DifferentialButton.Value = false;
                
                app.TimesecButton.Value = true;
                app.TemperatureCButton_X.Value = false;
                
                app.HeatFlowRateWgButton.Value = false;
                app.TemperatureCButton_Y.Value = true;
            end
            
            updateUIStates(app);
            
        end

        % Value changed function: ReferenceButton
        function ReferenceButtonValueChanged(app, event)
            value = app.ReferenceButton.Value;
            if value
                app.DifferentialButton.Value = false;
            end
            
            updateUIStates(app);
            
        end

        % Value changed function: TestSampButton
        function TestSampButtonValueChanged(app, event)
            value = app.TestSampButton.Value;
            if value
                app.DifferentialButton.Value = false;
            end
            
            updateUIStates(app);
            
        end

        % Value changed function: TimesecButton
        function TimesecButtonValueChanged(app, event)
            value = app.TimesecButton.Value;
            if value
                app.TemperatureCButton_X.Value = false;
            else
                app.TemperatureCButton_X.Value = true;
            end
            
            updateUIStates(app);
            
        end

        % Value changed function: TemperatureCButton_X
        function TemperatureCButton_XValueChanged(app, event)
            value = app.TemperatureCButton_X.Value;
            if value
                app.TimesecButton.Value = false;
            else
                app.TimesecButton.Value = true;
            end
            
            updateUIStates(app);
            
        end

        % Value changed function: HeatFlowRateWgButton
        function HeatFlowRateWgButtonValueChanged(app, event)
            value = app.HeatFlowRateWgButton.Value;
            if value
                app.TemperatureCButton_Y.Value = false;
            else
                app.TemperatureCButton_Y.Value = true;
            end
            
            updateUIStates(app);
            
        end

        % Value changed function: TemperatureCButton_Y
        function TemperatureCButton_YValueChanged(app, event)
            value = app.TemperatureCButton_Y.Value;
            if value
                app.HeatFlowRateWgButton.Value = false;
            else
                app.HeatFlowRateWgButton.Value = true;
            end
            
            updateUIStates(app);
            
        end

        % Changes arrangement of the app based on UIFigure width
        function updateAppLayout(app, event)
            currentFigureWidth = app.UIFigure.Position(3);
            if(currentFigureWidth <= app.onePanelWidth)
                % Change to a 2x1 grid
                app.GridLayout.RowHeight = {480, 480};
                app.GridLayout.ColumnWidth = {'1x'};
                app.RightPanel.Layout.Row = 2;
                app.RightPanel.Layout.Column = 1;
            else
                % Change to a 1x2 grid
                app.GridLayout.RowHeight = {'1x'};
                app.GridLayout.ColumnWidth = {220, '1x'};
                app.RightPanel.Layout.Row = 1;
                app.RightPanel.Layout.Column = 2;
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [100 100 640 480];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.SizeChangedFcn = createCallbackFcn(app, @updateAppLayout, true);

            % Create GridLayout
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {220, '1x'};
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
            app.GridLayout2.RowHeight = {'1x', '1x', '2x', '2x', '2x'};
            app.GridLayout2.RowSpacing = 20;

            % Create LoadMATfileButton
            app.LoadMATfileButton = uibutton(app.GridLayout2, 'push');
            app.LoadMATfileButton.ButtonPushedFcn = createCallbackFcn(app, @LoadMATfileButtonPushed, true);
            app.LoadMATfileButton.BackgroundColor = [0.302 0.7451 0.9333];
            app.LoadMATfileButton.Layout.Row = 1;
            app.LoadMATfileButton.Layout.Column = [1 2];
            app.LoadMATfileButton.Text = 'Load MAT file';

            % Create DataSetSelectionPanel
            app.DataSetSelectionPanel = uipanel(app.GridLayout2);
            app.DataSetSelectionPanel.Title = 'Data Set Selection';
            app.DataSetSelectionPanel.BackgroundColor = [0.902 0.902 0.902];
            app.DataSetSelectionPanel.Layout.Row = 3;
            app.DataSetSelectionPanel.Layout.Column = [1 2];

            % Create GridLayout5
            app.GridLayout5 = uigridlayout(app.DataSetSelectionPanel);

            % Create DifferentialButton
            app.DifferentialButton = uibutton(app.GridLayout5, 'state');
            app.DifferentialButton.ValueChangedFcn = createCallbackFcn(app, @DifferentialButtonValueChanged, true);
            app.DifferentialButton.Text = 'Differential';
            app.DifferentialButton.Layout.Row = 1;
            app.DifferentialButton.Layout.Column = 1;
            app.DifferentialButton.Value = true;

            % Create TargetTempButton
            app.TargetTempButton = uibutton(app.GridLayout5, 'state');
            app.TargetTempButton.ValueChangedFcn = createCallbackFcn(app, @TargetTempButtonValueChanged, true);
            app.TargetTempButton.Text = 'Target Temp';
            app.TargetTempButton.Layout.Row = 1;
            app.TargetTempButton.Layout.Column = 2;

            % Create ReferenceButton
            app.ReferenceButton = uibutton(app.GridLayout5, 'state');
            app.ReferenceButton.ValueChangedFcn = createCallbackFcn(app, @ReferenceButtonValueChanged, true);
            app.ReferenceButton.Text = 'Reference';
            app.ReferenceButton.Layout.Row = 2;
            app.ReferenceButton.Layout.Column = 1;

            % Create TestSampButton
            app.TestSampButton = uibutton(app.GridLayout5, 'state');
            app.TestSampButton.ValueChangedFcn = createCallbackFcn(app, @TestSampButtonValueChanged, true);
            app.TestSampButton.Text = 'Test Samp';
            app.TestSampButton.Layout.Row = 2;
            app.TestSampButton.Layout.Column = 2;

            % Create XAxisDataPanel
            app.XAxisDataPanel = uipanel(app.GridLayout2);
            app.XAxisDataPanel.Title = 'X-Axis Data';
            app.XAxisDataPanel.BackgroundColor = [0.902 0.902 0.902];
            app.XAxisDataPanel.Layout.Row = 4;
            app.XAxisDataPanel.Layout.Column = [1 2];

            % Create GridLayout6
            app.GridLayout6 = uigridlayout(app.XAxisDataPanel);

            % Create TimesecButton
            app.TimesecButton = uibutton(app.GridLayout6, 'state');
            app.TimesecButton.ValueChangedFcn = createCallbackFcn(app, @TimesecButtonValueChanged, true);
            app.TimesecButton.Text = 'Time (sec)';
            app.TimesecButton.Layout.Row = 1;
            app.TimesecButton.Layout.Column = [1 2];

            % Create TemperatureCButton_X
            app.TemperatureCButton_X = uibutton(app.GridLayout6, 'state');
            app.TemperatureCButton_X.ValueChangedFcn = createCallbackFcn(app, @TemperatureCButton_XValueChanged, true);
            app.TemperatureCButton_X.Text = 'Temperature (C)';
            app.TemperatureCButton_X.Layout.Row = 2;
            app.TemperatureCButton_X.Layout.Column = [1 2];
            app.TemperatureCButton_X.Value = true;

            % Create YAxisDataPanel
            app.YAxisDataPanel = uipanel(app.GridLayout2);
            app.YAxisDataPanel.Title = 'Y-Axis Data';
            app.YAxisDataPanel.BackgroundColor = [0.902 0.902 0.902];
            app.YAxisDataPanel.Layout.Row = 5;
            app.YAxisDataPanel.Layout.Column = [1 2];

            % Create GridLayout7
            app.GridLayout7 = uigridlayout(app.YAxisDataPanel);

            % Create HeatFlowRateWgButton
            app.HeatFlowRateWgButton = uibutton(app.GridLayout7, 'state');
            app.HeatFlowRateWgButton.ValueChangedFcn = createCallbackFcn(app, @HeatFlowRateWgButtonValueChanged, true);
            app.HeatFlowRateWgButton.Text = 'Heat Flow Rate (W/g)';
            app.HeatFlowRateWgButton.Layout.Row = 1;
            app.HeatFlowRateWgButton.Layout.Column = [1 2];
            app.HeatFlowRateWgButton.Value = true;

            % Create TemperatureCButton_Y
            app.TemperatureCButton_Y = uibutton(app.GridLayout7, 'state');
            app.TemperatureCButton_Y.ValueChangedFcn = createCallbackFcn(app, @TemperatureCButton_YValueChanged, true);
            app.TemperatureCButton_Y.Text = 'Temperature (C)';
            app.TemperatureCButton_Y.Layout.Row = 2;
            app.TemperatureCButton_Y.Layout.Column = [1 2];

            % Create RightPanel
            app.RightPanel = uipanel(app.GridLayout);
            app.RightPanel.Layout.Row = 1;
            app.RightPanel.Layout.Column = 2;

            % Create GridLayout3
            app.GridLayout3 = uigridlayout(app.RightPanel);
            app.GridLayout3.ColumnWidth = {'1x'};
            app.GridLayout3.RowHeight = {'1x'};
            app.GridLayout3.Padding = [5 5 5 5];

            % Create UIAxes
            app.UIAxes = uiaxes(app.GridLayout3);
            title(app.UIAxes, 'Differential Heat Flow Rate vs. Temperature')
            xlabel(app.UIAxes, 'Temperature (C)')
            ylabel(app.UIAxes, 'Heat Flow Rate (W/g)')
            app.UIAxes.XGrid = 'on';
            app.UIAxes.YGrid = 'on';
            app.UIAxes.Layout.Row = 1;
            app.UIAxes.Layout.Column = 1;

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = DSC_Data_Analysis_UI_exported

            runningApp = getRunningApp(app);

            % Check for running singleton app
            if isempty(runningApp)

                % Create UIFigure and components
                createComponents(app)

                % Register the app with App Designer
                registerApp(app, app.UIFigure)
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