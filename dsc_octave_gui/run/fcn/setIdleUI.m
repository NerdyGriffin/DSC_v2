%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  setIdleUI()
%Function setIdleUI()
%@end deftypefn

function ret = setIdleUI (dlg)
  set(dlg.StartExperimentButton, 'enable', 'on');
  set(dlg.StartExperimentButton, 'enable', 'on');
  set(dlg.StopExperimentButton, 'enable', 'off');
  set(dlg.LoadConfigFileButton, 'enable', 'on');
  set(dlg.SyncToArduinoButton, 'enable', 'on');
  %set(dlg.StartTempEditField, 'editable', 'on');
  %set(dlg.EndTempEditField, 'editable', 'on');
  %set(dlg.RateEditField, 'editable', 'on');
  %set(dlg.HoldTimeEditField, 'editable', 'on');
  set(dlg.SetSerialPortButton, 'enable', 'on');
  %set(dlg.SerialPortComboBox, 'editable', 'on');

  ret = 0;
endfunction
