%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  sendPIDGains()
%Function sendPIDGains()
%@end deftypefn

function ret = sendPIDGains (dlg)
    updateProgressDlg(dlg, 'Sending PID Gains to Arduino...');

    % Send the PID gain constants via the serial bus
    flush(dlg.Arduino);
    write(dlg.Arduino, 'p', 'char');

    write(dlg.Arduino, string(dlg.Data.Kp), 'string');
    write(dlg.Arduino, ' ', 'char');
    write(dlg.Arduino, string(dlg.Data.Ki), 'string');
    write(dlg.Arduino, ' ', 'char');
    write(dlg.Arduino, string(dlg.Data.Kd), 'string');

    ret = 0;
endfunction
