## -*- texinfo -*-
## @deftypefn {} {} arduinoNano (@var{parameterList})
## Create a object for represent a Arduino Nano board conected in serialPort
##
## @example
## retObj = arduinoNano('serialPort', '/dev/ttyUSB0')
## retObj = aNano('serialPort', 'COM'input'')
## @end example
##
## @noindent
## @end deftypefn

function retObj = arduinoNano(varargin)  
  %% Si esta instalado el paquete de instrumentacion lo cargamos.
  %% en caso contrario mostramos un mensaje indicando que debe instalarse
  %% el paquete.
  havePkg = pkg('list', 'instrument-control');
  if(isempty(havePkg)) 
    disp "This package require a instrument-control package for run. ";
    disp "Please install instrument-control package from the forge repo to continue.";
    error('It was not possible to load the instrument-control package');
    retObj = 0;
    return;
  else 
    pkg('load', 'instrument-control')
  endif
  %% Verificamos que se pueda acceder al puerto serie.
  if (exist("serial") != 3)
    disp "Serial port unsupported";
    return;
  endif
  
  if (length (varargin) < 2 || rem (length (varargin), 2) != 0)
    error ("set: expecting property/value pairs");
  endif
  
  retObj.serialPort = ''; %serialPortName;
  retObj.serialDev = struct(); %serial(serialPortName, 115200, 1);
  retObj = class(retObj, "arduinoNano");    
 
  defPins = 0;
  pinNum = [];
  pinType = cellstr("");
  while (length (varargin) > 1)
    prop = varargin{1};
    val = varargin{2};
    varargin(1:2) = [];
    if (strcmp(prop, "serialPort"))
      retObj.serialPort = val;     
    endif
    
    for ind=2:19
      strTest = ["pinDir" num2str(ind)];
      if(strcmp(strTest, prop))
        defPins = defPins + 1;
        pinNum(defPins) = ind;
        pinType(1, defPins) = val;
      endif
    endfor
       
  endwhile

  parentWnd = gcf();
  wnd = serialCfg();
  setappdata(wnd.figure, "parentWnd", parentWnd);
  waitfor(wnd.figure);
  if length(getappdata(parentWnd, "serialPort")) > 0
    if isunix()
      retObj.serialPort = ["/dev/" getappdata(parentWnd, "serialPort")];
    elseif ismac()
      retObj.serialPort = ["/dev/" getappdata(parentWnd, "serialPort")];
    else
      retObj.serialPort = ["\\\\.\\" getappdata(parentWnd, "serialPort")];
    endif
    disp(["Using: " retObj.serialPort]);
  else
     error('Serial port selecction canceled.');
  endif

  retObj.serialDev = serial(retObj.serialPort, 9600);      
  pause(2.5);
  srl_flush(retObj.serialDev);

  for ind=1:defPins    
    aNanoSetPinType(retObj, pinNum(ind), "digital", pinType(1,ind));
  endfor  

endfunction
