%       ___           ___           ___           ___ 
%      /\  \         /\  \         /\  \         /\__\
%     /::\  \       /::\  \       /::\  \       /:/  /
%    /:/\:\  \     /:/\ \  \     /:/\:\  \     /:/  / 
%   /::\~\:\  \   _\:\~\ \  \   /::\~\:\  \   /:/  /  
%  /:/\:\ \:\__\ /\ \:\ \ \__\ /:/\:\ \:\__\ /:/__/   
%  \/__\:\/:/  / \:\ \:\ \/__/ \/_|::\/:/  / \:\  \   
%       \::/  /   \:\ \:\__\      |:|::/  /   \:\  \  
%       /:/  /     \:\/:/  /      |:|\/__/     \:\  \ 
%      /:/  /       \::/  /       |:|  |        \:\__\
%      \/__/         \/__/         \|__|         \/__/
%   |------------Autonomous Space Robotics Lab -------|
%   /name   CompareFrameToReferenceFrame
%   /brief  Compares each frame from a particular trial to a frame from a
%           reference set of image stacks.
function CompareFrameToReferenceFrame()
    DEBUG = 0;

    %Set root dirs from compensated and uncompensated image stacks
    compRootDir = '/mnt/data/Datasets/Features/RANSAC-Comp/';
    distortedRootDir = '/mnt/data/Datasets/Features/02-Feb-13/';
    
    %Set dirs from which to load imagestacks
    referenceDir = '/mnt/data/Datasets/Features/02-Feb-13/ImageStacks/smoothturning-0.0-0.25-13:06/0001/';
    compDir = [compRootDir 'ImageStacks/']
    distortedDir = [distortedRootDir 'ImageStacks/']
    
    %Get directories containing image stacks
    distortedDirs = dir([distortedDir 'smooth*']);
    compDirs = dir([compDir 'smooth*']);
    
    %fileList = dir([distortedRootDir 'Bags/Processed/Turning/' 'smooth*']);
    
    %Load groundtruth data for reference trial
    fileList = dir([distortedRootDir 'Bags/Processed/Turning/' 'smoothturning-0-0.25-13:06*']);
    [T_ref, t_ref,~,~] = ParseViconGroundTruth([distortedRootDir 'Bags/Processed/Turning/'],fileList,0,1);
    
    T_ref_out = FindFrameTransforms(referenceDir,T_ref,t_ref)
    
    
    %Loop over each directory and process file
    for dirIndex = 1 : size(distortedDirs,1)
        distortedFilePath = distortedDirs (dirIndex);
        compFilePath = compDirs(dirIndex);
        
        %Get image stacks in dir
        distortedImageStacks = dir([distortedDir distortedFilePath.name '/0001/*.asa']);
        compedImageStacks = dir([compDir compFilePath.name '/0001/*.asa']);
        
        compSum = 0;
        distortSum = 0;
        if(size(compedImageStacks,1)>0)
            for i = 1 : min(size(compedImageStacks,1),size(compedImageStacks,1)) - 3
                %Load files to compare
                compensatedScan = loadAsrlMatArchive([compDir compFilePath.name '/0001/' compedImageStacks(i).name]);                
                distortedScan  = loadAsrlMatArchive([distortedDir distortedFilePath.name '/0001/' distortedImageStacks(i).name]);
                
                %Get matching frame from reference series
                T = zeros(4,4);
                idx = FindReferenceFrame(referenceDir,T_vicon_v,T);
                
                %Compare comped and uncomped scans
            end
        end
    end
end