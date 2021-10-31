%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  setRunningUI()
%Function setRunningUI()
%@end deftypefn

function ret = setRunningUI (dlg)
  set(dlg.StartExperimentButton, 'enable', 'off');
  set(dlg.StopExperimentButton, 'enable', 'on');
  set(dlg.LoadConfigFileButton, 'enable', 'off');
  set(dlg.SyncToArduinoButton, 'enable', 'off');
  %set(dlg.StartTempEditField, 'editable', 'off');
  %set(dlg.EndTempEditField, 'editable', 'off');
  %set(dlg.RateEditField, 'editable', 'off');
  %set(dlg.HoldTimeEditField, 'editable', 'off');
  set(dlg.SetSerialPortButton, 'enable', 'off');
  %set(dlg.SerialPortComboBox, 'editable', 'off');

  ret = 0;
endfunction
