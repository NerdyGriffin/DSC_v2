%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  updateLiveData()
%Function updateLiveData()
%@end deftypefn

function ret = updateLiveData (dlg, elapsedTime, targetTemp, ...
    refTemp, sampTemp, refCurrent, sampCurrent, ...
    refDutyCycle, sampDutyCycle)
  drawnow limitrate nocallbacks

  % Convert from milliseconds to seconds
  dlg.ElapsedTimesecEditField.Value = elapsedTime;

  dlg.TargetTempCEditField.Value = targetTemp;

  dlg.TemperatureCEditField.Value = refTemp;
  dlg.CurrentmAEditField.Value = refCurrent;
  dlg.PWMDutyCycleEditField.Value = refDutyCycle;

  dlg.TemperatureCEditField_2.Value = sampTemp;
  dlg.CurrentmAEditField_2.Value = sampCurrent;
  dlg.PWMDutyCycleEditField_2.Value = sampDutyCycle;

  drawnow limitrate

  ret = 0;
endfunction
