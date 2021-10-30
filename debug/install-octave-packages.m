% Run this script from Octave to install the packages required for this project
packagesToInstall = {"instrument-ontrol"};

for packageToInstall = packagesToInstall
    packageNotInstalled = true;
    [dummy, installedPackages] = pkg('list');
    for i = 1:length(installedPackages)
      installedPackageNames{i} = installedPackages{i}.name;
    endfor
    if !any(strcmp(packageToInstall, installedPackageNames))
      packageName = packageToInstall{1};
      fprintf("Installing package '%s'\n", packageName);
      %pkg install -forge -verbose packageName
    endif
endfor
