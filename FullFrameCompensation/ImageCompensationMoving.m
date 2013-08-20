%             _____ _____  _      
%      /\    / ____|  __ \| |     
%     /  \  | (___ | |__) | |     
%    / /\ \  \___ \|  _  /| |     
%   / ____ \ ____) | | \ \| |____ 
%  /_/    \_\_____/|_|  \_\______|
%  Autonomous Space Robotics Lab
% 
% Description:
% Motion compensates Lidar images.
%
% Author:
% Erik E. Beerepoot
%
% Main steps of the algorithm:
%   1. Using vicon ground truth, estimate velocity for trajectory 
%   2. Iterating over the images, do:
%   2a. Grab (Az,El,Range) for each pixel
%   2b. Convert each to (x,y,z)
%   2c. Using the velocity estimated from vicon, and the compensated
%     measurement equation, construct new (Az,El,Range) for each pixel
%   2d. Consitute compensated image.
%   fin.
clear;
close all;

calcVel = 1;
dataPaths{1} = '/Users/erik/Dataset/12-May-12/';
dataPaths{2} = '/home/eeb/Datasets/Features/02-Feb-13/';
dataPaths{3} = '/home/eeb/Datasets/Features/12-May-12/';
dataPaths{4} = '/mnt/data/Datasets/Features/02-Feb-13/';
dataPaths{5} = '/mnt/data/Datasets/Features/12-May-12/';
dataPath = dataPaths{5};

outPaths{1} = '/home/eeb/Datasets/Features/Compensated/02-Feb-13/';
outPaths{2} = '/mnt/data/Datasets/Features/Compensated/12-May-12/';
outPath = outPaths{2};

if(calcVel)
    %1(i) Grab the transformation matrices
    %[T,t] = ParseVicon(0,0,[dataPath 'Bags/']);
    fileList = dir([dataPath 'Bags/Processed/turning/' 'smooth*']);
    [T,t] = ParseViconGroundTruth([dataPath 'Bags/Processed/turning/'],fileList,1,0);

    % In the ImageStacks folder, we find a folder for each trial 
    full_dirlist = dir([dataPath 'ImageStacks/turning/smooth*']);
    
    %Exclude regular files (only use dirs)
    isub = [full_dirlist(:).isdir];
    dirlist = {full_dirlist(isub).name}';
    
    % Exclude '.', '..', and mac resource forks. 
    dirlist(ismember(dirlist,{'.','..'})) = [];
   
    %Loop over the directories (skip "." and "..")
    endDirIndex = length(dirlist);
    if(endDirIndex < 1)
        disp('No files found in given data directory');
    end

    %Manually define offsets
    offsets = [200,4400; 800,4600; 660,3700; 650,2500;  1000,3000; 950,2300; 800,2075; 500,1650; 800,1850; 1350,2150];
    offsets = [1,0;1,0;1,0;1,0;1,0;1,0;1,0;1,0;1,0;1,0;];
    vel = zeros(6,endDirIndex);
    
    v_profile = 0;
    for dirIndex = 1:4%endDirIndex
        v(dirIndex) = CompensateImagesPiecewiseVelocity([dataPath 'ImageStacks/'],outPath,T,t,0.5,offsets,dirIndex);        
    end
end
