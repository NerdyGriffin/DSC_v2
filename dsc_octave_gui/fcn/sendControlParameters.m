%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  sendControlParameters()
%Function sendControlParameters()
%@end deftypefn

function ret = sendControlParameters (dlg)
  updateProgressDlg(dlg, 'Sending temperature control parameters to Arduino...');

  flush(dlg.Arduino);
  write(dlg.Arduino, 'l', 'char');

  write(dlg.Arduino, string(dlg.StartTempCEditField.Value), 'string');
  write(dlg.Arduino, ' ', 'char');
  write(dlg.Arduino, string(dlg.EndTempCEditField.Value), 'string');
  write(dlg.Arduino, ' ', 'char');
  write(dlg.Arduino, string(dlg.RateCminEditField.Value), 'string');
  write(dlg.Arduino, ' ', 'char');
  write(dlg.Arduino, string(dlg.HoldTimesecEditField.Value), 'string');

  ret = 0;
endfunction
