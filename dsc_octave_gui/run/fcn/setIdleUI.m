%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  setIdleUI()
%Function setIdleUI()
%@end deftypefn

function ret = setIdleUI (dlg)
  dlg.StartExperimentButton.Enable = 'on';
  dlg.StopExperimentButton.Enable = 'off';
  dlg.LoadConfigFileButton.Enable = 'on';
  dlg.ApplyExperimentParametersButton.Enable = 'on';
  dlg.StartTempCEditField.Editable = 'on';
  dlg.EndTempCEditField.Editable = 'on';
  dlg.RateCminEditField.Editable = 'on';
  dlg.HoldTimesecEditField.Editable = 'on';
  dlg.SetSerialPortButton.Enable = 'on';
  dlg.SerialPortEditField.Editable = 'on';

  ret = 0;
endfunction
