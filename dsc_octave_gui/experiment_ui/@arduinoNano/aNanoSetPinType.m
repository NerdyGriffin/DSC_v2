function aNanoSetPinType(obj, nroPin, tipo, varargin)
  v = [176, 0, nroPin, 0, 0, 0, 0, 0, 11];
  if (strcmp(class(obj), "arduinoNano") != 1)  
    error("Esta funcion ha sido definida para utilizar objetos de la clase arduinoNano");
  endif
  switch (tipo)
    case "digital" 
      v(2) = 1;
      val = varargin{1};
      if(strcmp(val, "output"))
        v(4) = 1;
      elseif(strcmp(val, "inputPullUp"))
        v(4) = 2;
      elseif(strcmp(val, "inputFloat") || strcmp(val, "input"))
        v(4) = 3;
      else
        error("Tipo de pin incorrecto");
      endif      
      v(8) = 0;
      for i=1:7
        v(8) = v(8) + v(i);
      endfor
      v(8) = mod(v(8), 256);      
      port = obj.serialDev;
      for i=1:9
        srl_write(port, uint8(v(i)));
      endfor
    otherwise
      error("Tipo de pin no soportado");
  endswitch
endfunction
