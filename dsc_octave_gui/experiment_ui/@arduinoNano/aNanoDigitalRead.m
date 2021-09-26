% aNanoDigitalRead(obj, pin)
% obj: the arduino controll used for communication
% pin: pin number to read.  Valid identification of pin are in range 2 to 19.
% Return: the value result of read operation
% El paquete de datos, para pines digitales, se construye como:
% 0xB0, CMD = 3, pin, 0, 0, 0, 0, suma, 0x0b
% Return 1 byte with the logic state of pin

function ret = aNanoDigitalRead(obj, pin)
  v = [176, 3, pin, 0, 0, 0, 0, 0, 11];

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
  ret =  srl_read(port, 1);
endfunction
