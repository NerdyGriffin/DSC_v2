% aNanoAnalogRead(obj, pin)
% obj: the arduino controll used for communication
% pin: pin number to read. 
% Return: the value result of read operation
% Valid pin values are 14 (A0) to 21 (A7)
% If use  1 (as pin identification), the firmware return a sin(x) signal 
% with values from 0 to 1024. With each reading the next value in the sequence will be taken.
% The package send is formed as:
% 0xB0, CMD = 2, nroPin, 0, 0, 0, 0, suma, 0x0b
% Retorna 2 bytes representando el valor resultante de la conversión.
function ret = aNanoAnalogRead(obj, pin)
  v = [176, 2, pin, 0, 0, 0, 0, 0, 11];
  
  v(8) = 0;
  for i=1:7
    v(8) = v(8) + v(i);
  endfor
  v(8) = mod(v(8), 256);      
  port = obj.serialDev;
  srl_flush(port);
  for i=1:9
    srl_write(port, uint8(v(i)));
  endfor
  b0 = srl_read(port, 1);
  b1 = srl_read(port, 1);
  ret =  ((uint16(b1)*256) + uint16(b0));
endfunction
