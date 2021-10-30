## -*- texinfo -*-
## @deftypefn  {} {} dummy()
##
## This is a dummy function documentation. This file have a lot functions
## and each one have a little documentation. This text is to avoid a warning when
## install this file as part of package.
## @end deftypefn
##
## Set the graphics toolkit and force read this file as script file (not a function file).
##
graphics_toolkit qt;
##


##
##
## Begin callbacks definitions 
##

## @deftypefn  {} {} StartExperimentButton_doIt (@var{src}, @var{data}, @var{Experiment_UI})
##
## Define a callback for default action of StartExperimentButton control.
##
## @end deftypefn
function StartExperimentButton_doIt(src, data, Experiment_UI)

% This code will be executed when user click the button control.
% As default, all events are deactivated, to activate must set the
% propertie 'generateCallback' from the properties editor
end

## @deftypefn  {} {} StopExperimentButton_doIt (@var{src}, @var{data}, @var{Experiment_UI})
##
## Define a callback for default action of StopExperimentButton control.
##
## @end deftypefn
function StopExperimentButton_doIt(src, data, Experiment_UI)

% This code will be executed when user click the button control.
% As default, all events are deactivated, to activate must set the
% propertie 'generateCallback' from the properties editor
end

## @deftypefn  {} {} LoadConfigFileButton_doIt (@var{src}, @var{data}, @var{Experiment_UI})
##
## Define a callback for default action of LoadConfigFileButton control.
##
## @end deftypefn
function LoadConfigFileButton_doIt(src, data, Experiment_UI)

% This code will be executed when user click the button control.
% As default, all events are deactivated, to activate must set the
% propertie 'generateCallback' from the properties editor
end

## @deftypefn  {} {} SyncToArduinoButton_doIt (@var{src}, @var{data}, @var{Experiment_UI})
##
## Define a callback for default action of SyncToArduinoButton control.
##
## @end deftypefn
function SyncToArduinoButton_doIt(src, data, Experiment_UI)

% This code will be executed when user click the button control.
% As default, all events are deactivated, to activate must set the
% propertie 'generateCallback' from the properties editor
end

## @deftypefn  {} {} Button_5_doIt (@var{src}, @var{data}, @var{Experiment_UI})
##
## Define a callback for default action of Button_5 control.
##
## @end deftypefn
function Button_5_doIt(src, data, Experiment_UI)

% This code will be executed when user click the button control.
% As default, all events are deactivated, to activate must set the
% propertie 'generateCallback' from the properties editor
end

 
## @deftypefn  {} {@var{ret} = } show_Experiment_UI()
##
## Create windows controls over a figure, link controls with callbacks and return 
## a window struct representation.
##
## @end deftypefn
function ret = show_Experiment_UI()
  _scrSize = get(0, "screensize");
  _xPos = (_scrSize(3) - 1264)/2;
  _yPos = (_scrSize(4) - 681)/2;
   Experiment_UI = figure ( ... 
	'Color', [0.941 0.941 0.941], ...
	'Position', [_xPos _yPos 1264 681], ...
	'resize', 'off', ...
	'windowstyle', 'normal', ...
	'MenuBar', 'none');
	 set(Experiment_UI, 'visible', 'off');
  Main_Plot = axes( ...
	'Units', 'pixels', ... 
	'parent',Experiment_UI, ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'Position', [240 41 820 600]);
  StartExperimentButton = uicontrol( ...
	'parent',Experiment_UI, ... 
	'Style','pushbutton', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'Position', [5 632 180 44], ... 
	'String', 'Start Experiment', ... 
	'TooltipString', '');
  StopExperimentButton = uicontrol( ...
	'parent',Experiment_UI, ... 
	'Style','pushbutton', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'Position', [5 582 180 44], ... 
	'String', 'Stop Experiment', ... 
	'TooltipString', '');
  LoadConfigFileButton = uicontrol( ...
	'parent',Experiment_UI, ... 
	'Style','pushbutton', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'Position', [5 532 180 44], ... 
	'String', 'Load Config File', ... 
	'TooltipString', '');
  SyncToArduinoButton = uicontrol( ...
	'parent',Experiment_UI, ... 
	'Style','pushbutton', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'Position', [5 482 180 44], ... 
	'String', 'Sync to Arduino', ... 
	'TooltipString', '');
  ComboBox_1 = uicontrol( ...
	'parent',Experiment_UI, ... 
	'Style','popupmenu', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'Position', [110 56 70 25], ... 
	'String', 'COM1|COM2|COM3', ... 
	'TooltipString', '');
  Serial_Port_Label = uicontrol( ...
	'parent',Experiment_UI, ... 
	'Style','text', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [10 60 61 16], ... 
	'String', 'Serial Port', ... 
	'TooltipString', '');
  Button_5 = uicontrol( ...
	'parent',Experiment_UI, ... 
	'Style','pushbutton', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'Position', [5 2 180 44], ... 
	'String', 'Set Serial Port', ... 
	'TooltipString', '');
  GroupPanel_1 = uipanel( ...
	'parent',Experiment_UI, ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.863 0.863 0.863], ... 
	'BorderWidth', 1, ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'Position', [5 316 180 145], ... 
	'title', 'Experiment Parameters', ... 
	'TitlePosition', 'lefttop');
  Start_Temp_Label = uicontrol( ...
	'parent',GroupPanel_1, ... 
	'Style','text', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [10 104 84 16], ... 
	'String', 'Start Temp (C)', ... 
	'TooltipString', '');
  Edit_1 = uicontrol( ...
	'parent',GroupPanel_1, ... 
	'Style','edit', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [1.000 1.000 1.000], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [110 98 60 22], ... 
	'String', '0', ... 
	'TooltipString', '');
  End_Temp_Label = uicontrol( ...
	'parent',GroupPanel_1, ... 
	'Style','text', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [10 74 79 16], ... 
	'String', 'End Temp (C)', ... 
	'TooltipString', '');
  Edit_2 = uicontrol( ...
	'parent',GroupPanel_1, ... 
	'Style','edit', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [1.000 1.000 1.000], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [110 68 60 22], ... 
	'String', '0', ... 
	'TooltipString', '');
  Rate_Label = uicontrol( ...
	'parent',GroupPanel_1, ... 
	'Style','text', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [10 44 73 16], ... 
	'String', 'Rate (C/min)', ... 
	'TooltipString', '');
  Hold_Time_Label = uicontrol( ...
	'parent',GroupPanel_1, ... 
	'Style','text', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [10 14 91 16], ... 
	'String', 'Hold Time (sec)', ... 
	'TooltipString', '');
  Edit_3 = uicontrol( ...
	'parent',GroupPanel_1, ... 
	'Style','edit', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [1.000 1.000 1.000], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [110 38 60 22], ... 
	'String', '0', ... 
	'TooltipString', '');
  Edit_4 = uicontrol( ...
	'parent',GroupPanel_1, ... 
	'Style','edit', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [1.000 1.000 1.000], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [110 8 60 22], ... 
	'String', '0', ... 
	'TooltipString', '');
  Edit_5 = uicontrol( ...
	'parent',Experiment_UI, ... 
	'Style','edit', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [1.000 1.000 1.000], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [1195 644 60 22], ... 
	'String', '0.00', ... 
	'TooltipString', '');
  Edit_6 = uicontrol( ...
	'parent',Experiment_UI, ... 
	'Style','edit', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [1.000 1.000 1.000], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [1195 609 60 22], ... 
	'String', '0.00', ... 
	'TooltipString', '');
  GroupPanel_2 = uipanel( ...
	'parent',Experiment_UI, ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'BorderWidth', 1, ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'Position', [1075 466 180 115], ... 
	'title', 'Reference Sample Live Data', ... 
	'TitlePosition', 'lefttop');
  Edit_7 = uicontrol( ...
	'parent',GroupPanel_2, ... 
	'Style','edit', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [1.000 1.000 1.000], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [110 68 60 22], ... 
	'String', '0.00', ... 
	'TooltipString', '');
  Edit_8 = uicontrol( ...
	'parent',GroupPanel_2, ... 
	'Style','edit', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [1.000 1.000 1.000], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [110 38 60 22], ... 
	'String', '0.00', ... 
	'TooltipString', '');
  Edit_9 = uicontrol( ...
	'parent',GroupPanel_2, ... 
	'Style','edit', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [1.000 1.000 1.000], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [110 8 60 22], ... 
	'String', '0.00', ... 
	'TooltipString', '');
  Ref_Temp_Label = uicontrol( ...
	'parent',GroupPanel_2, ... 
	'Style','text', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [10 74 92 16], ... 
	'String', 'Temperature (C)', ... 
	'TooltipString', '');
  Ref_Cur_Label = uicontrol( ...
	'parent',GroupPanel_2, ... 
	'Style','text', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [10 44 74 16], ... 
	'String', 'Current (mA)', ... 
	'TooltipString', '');
  Ref_PWM_Label = uicontrol( ...
	'parent',GroupPanel_2, ... 
	'Style','text', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [10 14 101 16], ... 
	'String', 'PWM Duty Cycle', ... 
	'TooltipString', '');
  GroupPanel_3 = uipanel( ...
	'parent',Experiment_UI, ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'BorderWidth', 1, ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'Position', [1075 316 180 115], ... 
	'title', 'Test Sample Live Data', ... 
	'TitlePosition', 'lefttop');
  Edit_10 = uicontrol( ...
	'parent',GroupPanel_3, ... 
	'Style','edit', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [1.000 1.000 1.000], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [110 68 60 22], ... 
	'String', '0.00', ... 
	'TooltipString', '');
  Edit_11 = uicontrol( ...
	'parent',GroupPanel_3, ... 
	'Style','edit', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [1.000 1.000 1.000], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [110 38 60 22], ... 
	'String', '0.00', ... 
	'TooltipString', '');
  Edit_12 = uicontrol( ...
	'parent',GroupPanel_3, ... 
	'Style','edit', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [1.000 1.000 1.000], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [110 8 60 22], ... 
	'String', '0.00', ... 
	'TooltipString', '');
  Samp_Temp_Label = uicontrol( ...
	'parent',GroupPanel_3, ... 
	'Style','text', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [10 74 92 16], ... 
	'String', 'Temperature (C)', ... 
	'TooltipString', '');
  Samp_Cur_Label = uicontrol( ...
	'parent',GroupPanel_3, ... 
	'Style','text', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [10 44 74 16], ... 
	'String', 'Current (mA)', ... 
	'TooltipString', '');
  Samp_PWM_Label = uicontrol( ...
	'parent',GroupPanel_3, ... 
	'Style','text', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [10 14 101 16], ... 
	'String', 'PWM Duty Cycle', ... 
	'TooltipString', '');
  Time_Label = uicontrol( ...
	'parent',Experiment_UI, ... 
	'Style','text', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [1075 650 112 16], ... 
	'String', 'Elapsed Time (sec)', ... 
	'TooltipString', '');
  Target_Temp_Label = uicontrol( ...
	'parent',Experiment_UI, ... 
	'Style','text', ... 
	'Units', 'pixels', ... 
	'BackgroundColor', [0.941 0.941 0.941], ... 
	'FontAngle', 'normal', ... 
	'FontName', 'Arial', ... 
	'FontSize', 10, 'FontUnits', 'points', ... 
	'FontWeight', 'normal', ... 
	'ForegroundColor', [0.000 0.000 0.000], ... 
	'HorizontalAlignment', 'left', ... 
	'Position', [1075 615 91 16], ... 
	'String', 'Target Temp (C)', ... 
	'TooltipString', '');

  Experiment_UI = struct( ...
      'figure', Experiment_UI, ...
      'Main_Plot', Main_Plot, ...
      'StartExperimentButton', StartExperimentButton, ...
      'StopExperimentButton', StopExperimentButton, ...
      'LoadConfigFileButton', LoadConfigFileButton, ...
      'SyncToArduinoButton', SyncToArduinoButton, ...
      'ComboBox_1', ComboBox_1, ...
      'Serial_Port_Label', Serial_Port_Label, ...
      'Button_5', Button_5, ...
      'GroupPanel_1', GroupPanel_1, ...
      'Start_Temp_Label', Start_Temp_Label, ...
      'Edit_1', Edit_1, ...
      'End_Temp_Label', End_Temp_Label, ...
      'Edit_2', Edit_2, ...
      'Rate_Label', Rate_Label, ...
      'Hold_Time_Label', Hold_Time_Label, ...
      'Edit_3', Edit_3, ...
      'Edit_4', Edit_4, ...
      'Edit_5', Edit_5, ...
      'Edit_6', Edit_6, ...
      'GroupPanel_2', GroupPanel_2, ...
      'Edit_7', Edit_7, ...
      'Edit_8', Edit_8, ...
      'Edit_9', Edit_9, ...
      'Ref_Temp_Label', Ref_Temp_Label, ...
      'Ref_Cur_Label', Ref_Cur_Label, ...
      'Ref_PWM_Label', Ref_PWM_Label, ...
      'GroupPanel_3', GroupPanel_3, ...
      'Edit_10', Edit_10, ...
      'Edit_11', Edit_11, ...
      'Edit_12', Edit_12, ...
      'Samp_Temp_Label', Samp_Temp_Label, ...
      'Samp_Cur_Label', Samp_Cur_Label, ...
      'Samp_PWM_Label', Samp_PWM_Label, ...
      'Time_Label', Time_Label, ...
      'Target_Temp_Label', Target_Temp_Label);


  set (StartExperimentButton, 'callback', {@StartExperimentButton_doIt, Experiment_UI});
  set (StopExperimentButton, 'callback', {@StopExperimentButton_doIt, Experiment_UI});
  set (LoadConfigFileButton, 'callback', {@LoadConfigFileButton_doIt, Experiment_UI});
  set (SyncToArduinoButton, 'callback', {@SyncToArduinoButton_doIt, Experiment_UI});
  set (Button_5, 'callback', {@Button_5_doIt, Experiment_UI});
  dlg = struct(Experiment_UI);

%
% The source code writed here will be executed when
% windows load. Work like 'onLoad' event of other languages.
%

pkg load instrument-control

% Initialize the struct to prevent errors
Data = struct('Kp', 0, 'Ki', 0, 'Kd', 0, ...
    'startTemp', 0, 'endTemp', 0, 'rampUpRate', 0, ...
    'holdTime', 0, 'startDateTime', datestr(clock()));

dlg.Data = Data;

%dlg.SerialPortEditField.Value = '';

initializeSerialPort(dlg);

% Set title and labels for plot
title(dlg.Main_Plot, 'Temperature vs. Time');
xlabel(dlg.Main_Plot, 'Time (sec)');
ylabel(dlg.Main_Plot, 'Temperature (C)');

%setIdleUI(dlg);

% DEBUG
elapsedTimeArray = [0, 1];
targetTempArray = [0, 0];
refTempArray = [0, 1];
sampTempArray = [0, -1];

refreshLivePlot (dlg, elapsedTimeArray, targetTempArray, refTempArray, sampTempArray);


  set(Experiment_UI.figure, 'visible', 'on');
  ret = Experiment_UI;
end

