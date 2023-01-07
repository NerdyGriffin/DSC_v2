%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  sendControlParameters()
%Function sendControlParameters()
%@end deftypefn

function ret = sendControlParameters (dlg, fraction = 0)
  dlg.SharedProgressDlg = updateProgressDlg(dlg, fraction, 'Sending temperature control parameters to Arduino...');

  flush(dlg.Arduino);
  write(dlg.Arduino, 'l', 'char');

  write(dlg.Arduino, string(dlg.StartTempEditField.Value), 'string');
  write(dlg.Arduino, ' ', 'char');
  write(dlg.Arduino, string(dlg.EndTempEditField.Value), 'string');
  write(dlg.Arduino, ' ', 'char');
  write(dlg.Arduino, string(dlg.RateEditField.Value), 'string');
  write(dlg.Arduino, ' ', 'char');
  write(dlg.Arduino, string(dlg.HoldTimeEditField.Value), 'string');

  ret = 0;
endfunction
