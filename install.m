% ==========================
%  MULTIBODY LAB for MATLAB 
% ==========================
% 
% This script adds the '/source' and '/models' subfolders to the MATLAB path
% and saves the path for future MATLAB session. (There is no need to install it every time)

install_multibody_library();

function install_multibody_library()
    fprintf(...
        "  ==========================\n"+...
        "   MULTIBODY LAB for MATLAB \n"+...
        "  ==========================\n");
    fprintf("  Installing library...\n");
    
    rootdir = erase(mfilename("fullpath"), "install");
    currentpath = split(string(path), ':');
    alreadyinstalled = any(contains(currentpath, rootdir));
    
    if alreadyinstalled
        fprintf("  Already installed. Re-installing...\n")
    end
    
    addpath(rootdir + "models");
    addpath(rootdir + "source");
    
    fprintf(...
        "  Done.\n"+...
        "  Folders added to MATLAB's path:\n"+...
        "    " + rootdir + "models\n"+...
        "    " + rootdir + "source\n");
    
    savepath
end