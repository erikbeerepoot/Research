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

    %Hand computed reference indexes
    referenceIndexes(1,:) = [1 ,1:86]
    referenceIndexes(2,:) = [1,1,1,1,1,6,8,8,9,11,12,13,14,15,16,17,19,20,21,22,23,24,26,27,29,31,32,34,35,36,38,40,42,43,44,45,46,47,49,50,51,52,53,54,55,57,58,zeros(1,40)];
    referenceIndexes(3,:) = [3,3,3,6,7,9,10,13,14,15,17,19,20,22,24,26,27,28,30,33,34,36,38,40,41,43,45,47,49,51,53,54,57,58,60,61,62,64,66,68,71,73,75,zeros(1,44)];
    referenceIndexes(4,:) = [1,1,4,7,8,10,12,14,16,18,20,22,24,26,28,30,33,35,38,40,43,45,48,51,52,54,56,zeros(1,60)];
    referenceIndexes(5,:) = [1,2,1,6,7,9,11,13,15,17,19,21,23,26,28,30,32,35,37,40,42,45,47,49,51,53,55,59,61,zeros(1,58)];
    referenceIndexes(6,:) = [1,1,2,6,9,12,14,16,18,21,23,25,28,31,34,37,40,43,45,49,52,54,57,60,64,68,71,zeros(1,60)];
    referenceIndexes(7,:) = [1,7,7,9,12,14,17,19,22,25,29,31,34,37,42,45,50,52,55,59,65,69,73,zeros(1,64)];
    referenceIndexes(8,:) = [3,3,3,3,8,12,15,19,22,25,30,34,37,41,45,49,53,55,62,zeros(1,68)];
    referenceIndexes(9,:) = [1,1,6,10,12,17,20,24,27,32,37,41,46,51,59,zeros(1,72)];
    referenceIndexes(10,:) =[3,5,6,11,14,18,22,27,31,36,41,46,50,53,60,zeros(1,72)];    
    v = [ -0.0911,-0.1010,-0.1088,-0.2012,-0.2060,-0.2330,-0.2527,-0.2933,-0.2953,-0.3629]
    
    DEBUG = 0;
    brightnessFactor = 256;
    autoComputeViewpoint = 0;
    
    %Set root dirs from compensated and uncompensated image stacks
    compRootDir = '/mnt/data/Datasets/Features/Compensated/02-Feb-13/';
    distortedRootDir = '/mnt/data/Datasets/Features/02-Feb-13/';
    
    %Set dirs from which to load imagestacks
    %referenceDir = '/mnt/data/Datasets/Features/02-Feb-13/ImageStacks/smoothturning-0.0-0.25-13:06/0001/';
    referenceDir = '/mnt/data/Datasets/Features/Compensated/02-Feb-13/ImageStacks/smoothturning-0.0-0.25-13:06/0001/';
    compDir = [compRootDir 'ImageStacks/']
    distortedDir = [distortedRootDir 'ImageStacks/']
    
    %Get directories containing image stacks
    distortedDirs = dir([distortedDir 'smooth*']);
    compDirs = dir([compDir 'smooth*']);

    %Load reference image stack directory listing
    referenceImageStacks = dir([referenceDir '*.asa']);
    
    %Load groundtruth data for reference trial
    fileList = dir([distortedRootDir 'Bags/Processed/Turning/' 'smoothturning-0.0-0.25-13:06*']);
    
    if(autoComputeViewpoint)
        [T_ref, t_ref,~,~] = ParseViconGroundTruth([distortedRootDir 'Bags/Processed/Turning/'],fileList,0,1);    
        T_ref_out = FindFrameTransforms(referenceDir,T_ref,t_ref);
    end
    
    %Loop over each directory and process file
    for dirIndex = 1 : size(distortedDirs,1)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Hack for garbage data
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %if(dirIndex==5)
        %    continue;
        %end
        distortedFilePath = distortedDirs (dirIndex);
        compFilePath = compDirs(dirIndex);
        
        %Get image stacks in dir
        distortedImageStacks = dir([distortedDir distortedFilePath.name '/0001/*.asa']);
        compedImageStacks = dir([compDir compFilePath.name '/0001/*.asa']);
        
        %Associate a transform with each frame (we only have to do this
        %once, since the robot location is the same 
        filteredFileName = regexp(distortedFilePath.name,'^([a-z]+)-([0-9]+.[0-9]+)-([0-9]+.[0-9]+)-','match')
        fileList = dir([distortedRootDir 'Bags/Processed/Turning/' filteredFileName{:} '*']);
        %[T_current, t_current,~,~] = ParseViconGroundTruth([distortedRootDir 'Bags/Processed/Turning/'],fileList(1),0,1);
        %T_compframes_out = FindFrameTransforms([distortedDir distortedFilePath.name '/0001/'],T_current,t_current);
        
        compSum = 0;
        distortSum = 0;
        if(size(compedImageStacks,1)>0)
            for frameIndex = 1 : min([size(distortedImageStacks,1),size(compedImageStacks,1),sum((referenceIndexes(dirIndex,:)>0))]) - 3
                %Load files to compare
                compensatedScan = loadAsrlMatArchive([compDir compFilePath.name '/0001/' compedImageStacks(frameIndex).name]);                
                distortedScan  = loadAsrlMatArchive([distortedDir distortedFilePath.name '/0001/' distortedImageStacks(frameIndex).name]);
                
                if(autoComputeViewpoint)
                    %Get matching frame from reference series (auto)
                    T = T_compframes_out(:,:,frameIndex);
                    idx = FindReferenceFrame(T_ref_out,T);
                else  
                    idx = referenceIndexes(dirIndex,frameIndex)
                end
                
                %Get reference image
                referenceFrame = loadAsrlMatArchive([referenceDir referenceImageStacks(idx).name]);
                close all;
                
                %Run comparison for different detectors with the same
                %descriptor (SURF)
                [surfCompScore(dirIndex,frameIndex) compTrackLength(dirIndex,frameIndex)] = CompareImagesByDescriptor(compensatedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'SURF');
                [surfDistortScore(dirIndex,frameIndex) distortTrackLength(dirIndex,frameIndex)] = CompareImagesByDescriptor(distortedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'SURF');
                
%                 [FASTcompScore(dirIndex,frameIndex) FASTcompTrackLength(dirIndex,frameIndex)] = CompareImagesByDescriptor(compensatedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'FAST');
%                 [FASTdistortScore(dirIndex,frameIndex) FASTdistortTrackLength(dirIndex,frameIndex)] = CompareImagesByDescriptor(distortedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'FAST');
%                 
%                 [HarrisCompScore(dirIndex,frameIndex) HarrisCompTrackLength(dirIndex,frameIndex)] = CompareImagesByDescriptor(compensatedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'Harris');
%                 [HarrisDistortScore(dirIndex,frameIndex) HarrisDistortTrackLength(dirIndex,frameIndex)] = CompareImagesByDescriptor(distortedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'Harris');
%                 
%                 [HarrisAffCompScore(dirIndex,frameIndex) HarrisAffCompTrackLength(dirIndex,frameIndex)] = CompareImagesByDescriptor(compensatedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'HarrisAffine');
%                 [HarrisAffDistortScore(dirIndex,frameIndex) HarrisAffDistortTrackLength(dirIndex,frameIndex)] = CompareImagesByDescriptor(distortedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'HarrisAffine');
%                 
%                 [MSERCompScore(dirIndex,frameIndex) MSERCompTrackLength(dirIndex,frameIndex)] = CompareImagesByDescriptor(compensatedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'MSER');
%                 [MSERDistortScore(dirIndex,frameIndex) MSERDistortTrackLength(dirIndex,frameIndex)] = CompareImagesByDescriptor(distortedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'MSER');
                
                %Compare comped and uncomped scans
                if(DEBUG && autoComputeViewpoint)
                	figure(999); clf;
                    hold on; axis equal; grid on;
                    %Label and plot origin frame
                    xlabel('x'); ylabel('y'); zlabel('z')
                    plotCoordinateFrame(rph2c([pi/2,-pi/2,pi]),[0,0,0]')

                    %Plot sparse reference frames
                    for refIdx = 1:6:size(T_ref_out,3)
                        plotCoordinateFrame(T_ref_out(1:3,1:3,refIdx),T_ref_out(1:3,4,refIdx),2); 
                    end
                    
                    %Plot current frame and matching reference frame
                    plotCoordinateFrame(T_ref_out(1:3,1:3,idx),T_ref_out(1:3,4,idx),2); 
                    plotCoordinateFrame(T(1:3,1:3),T(1:3,4),2); 
                    
                    %Mark current frame and matching reference frame                    
                    plot3([0 T_ref_out(1,4,idx)],[0 T_ref_out(2,4,idx)],[0 T_ref_out(3,4,idx)],'r')
                    plot3([0 T(1,4)],[0 T(2,4)],[0 T(3,4)],'b')
                    hold off;
                    pause(0.1);
                end                
            end
        end
    end
end