## -*- texinfo -*-
## @deftypefn  {} {@var{wnd} =} Experiment_UI ()
##
## Create and show the dialog, return a struct as representation of dialog.
##
## @end deftypefn
function wnd = Experiment_UI()
  Experiment_UI_def;
  wnd = show_Experiment_UI();
end
