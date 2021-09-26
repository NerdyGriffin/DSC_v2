% aNanoClose(obj)
% obj: the arduino controll used for communication
% This function close the connection with board. Its important 
% call this function to free port for other users or connections.
% This method must be called at end of of application. 
function ret = aNanoClose(obj)
  fclose(obj.serialDev);
endfunction
