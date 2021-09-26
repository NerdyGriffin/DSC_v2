% aNanoDigitalWrite(obj, pin, value)
% obj: the arduino controll used for communication
% pin: pin number to read.  Valid identification of pin are in range 2 to 19.
% value: 0 (false) or <>0 (true)
% The package send is:
% 0xB0, CMD = 5, nroPin, val, 0, 0, 0, suma, 0x0b
function aNanoDigitalWrite(obj, pin, val)
  v = [176, 5, pin, val, 0, 0, 0, 0, 11];

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
