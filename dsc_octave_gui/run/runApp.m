function ret = runApp()
  [dir, name, ext] = fileparts( mfilename('fullpathext') );
  global _dsc_octave_guiBasePath = dir;
  global _dsc_octave_guiImgPath = [dir filesep() 'img'];
  addpath([dir filesep() "libs" ]);
  addpath([dir filesep() "fcn" ]);
  addpath([dir filesep() "wnd" ]);
  waitfor(Experiment_UI().figure);
end
