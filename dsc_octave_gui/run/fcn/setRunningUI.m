%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  setRunningUI()
%Function setRunningUI()
%@end deftypefn

function ret = setRunningUI (dlg)
    dlg.StartExperimentButton.Enable = 'off';
    dlg.StopExperimentButton.Enable = 'on';
    dlg.LoadConfigFileButton.Enable = 'off';
    dlg.ApplyExperimentParametersButton.Enable = 'off';
    dlg.StartTempCEditField.Editable = 'off';
    dlg.EndTempCEditField.Editable = 'off';
    dlg.RateCminEditField.Editable = 'off';
    dlg.HoldTimesecEditField.Editable = 'off';
    dlg.SetSerialPortButton.Enable = 'off';
    dlg.SerialPortEditField.Editable = 'off';

    ret = 0;
endfunction
