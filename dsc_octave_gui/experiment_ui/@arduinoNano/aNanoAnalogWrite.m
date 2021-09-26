% aNanoAnalogWrite(obj, pin, value)
% obj: the arduino controll used for communication
% pin: pin number to write. No all pins can generate pwm signals. In arduino uno boards only 3, 5, 6, 9, 10, 11.
% Value: value of in range 0-255 to apply.
% The package send is:
% 0xB0, CMD = 4, nroPin, val, 0, 0, 0, suma, 0x0b

function aNanoAnalogWrite(obj, pin, val)
  v = [176, 4, pin, val, 0, 0, 0, 0, 11];
  
  v(8) = 0;
  for i=1:7
    v(8) = v(8) + v(i);
  endfor
  v(8) = mod(v(8), 256);      
  port = obj.serialDev;
  for i=1:9
    srl_write(port, uint8(v(i)));
  endfor

endfunction
