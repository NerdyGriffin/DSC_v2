## -*- texinfo -*-
## @deftypefn  {} {@var{wnd} =} serialCfg ()
##
## Create and show the dialog, return a struct as representation of dialog.
##
## @end deftypefn
function wnd = serialCfg()
  serialCfg_def;
  wnd = show_serialCfg();
end
