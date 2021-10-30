%-*- texinfo -*-
%@deftypefn {Function File} {@var{ret} =}  saveData()
%Function saveData()
%@end deftypefn

function ret = saveData (dlg, saveData)
    date_str = datestr(saveData.startDateTime, 'yyyy-mm-dd-HHMM');

    autoFilename = ['autosave/autoSaveData-', date_str];

    matFilename = [autoFilename, '.mat'];
    save(matFilename, '-struct', 'saveData');

    figureFilename = [autoFilename, '.fig'];
    saveAxesAsFigure(dlg, figureFilename);

    if isfile(matFilename)
        beep
        message = sprintf("Save file created: '%s'\n", matFilename);

        if isfile(figureFilename)
            message = sprintf("%s\n\nFigure file created: '%s'\n", message, figureFilename);
        end

        disp(message)
        uialert(dlg.UIFigure, message, 'Data Saved Successfully', 'Icon', 'success');
    end

    ret = 0;
endfunction
