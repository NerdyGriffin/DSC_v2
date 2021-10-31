%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  updateLiveData()
%Function updateLiveData()
%@end deftypefn

function ret = updateLiveData (dlg, elapsedTime, targetTemp, ...
    refTemp, sampTemp, refCurrent, sampCurrent, ...
    refDutyCycle, sampDutyCycle)
  drawnow limitrate nocallbacks

  % Convert from milliseconds to seconds
  set(dlg.ElapsedTimesecEditField, 'value', elapsedTime);

  set(dlg.TargetTempCEditField, 'value', targetTemp);

  set(dlg.TemperatureCEditField, 'value', refTemp);
  set(dlg.CurrentmAEditField, 'value', refCurrent);
  set(dlg.PWMDutyCycleEditField, 'value', refDutyCycle);

  set(dlg.TemperatureCEditField_2, 'value', sampTemp);
  set(dlg.CurrentmAEditField_2, 'value', sampCurrent);
  set(dlg.PWMDutyCycleEditField_2, 'value', sampDutyCycle);

  drawnow limitrate

  ret = 0;
endfunction
