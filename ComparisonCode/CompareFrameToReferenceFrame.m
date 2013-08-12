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
    brightnessFactor = 255;
    
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

    %Load reference image stack directory listing
    referenceImageStacks = dir([referenceDir '*.asa']);
    
    %Load groundtruth data for reference trial
    fileList = dir([distortedRootDir 'Bags/Processed/Turning/' 'smoothturning-0.0-0.25-13:06*']);
    [T_ref, t_ref,~,~] = ParseViconGroundTruth([distortedRootDir 'Bags/Processed/Turning/'],fileList,0,1);    
    T_ref_out = FindFrameTransforms(referenceDir,T_ref,t_ref);
        
    %Loop over each directory and process file
    for dirIndex = 1 : size(distortedDirs,1)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Hack for garbage data
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %if(dirIndex==4 || dirIndex==5)
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
        [T_current, t_current,~,~] = ParseViconGroundTruth([distortedRootDir 'Bags/Processed/Turning/'],fileList(1),0,1);
        T_compframes_out = FindFrameTransforms([distortedDir distortedFilePath.name '/0001/'],T_current,t_current);
        
        compSum = 0;
        distortSum = 0;
        if(size(compedImageStacks,1)>0)
            for frameIndex = 1 : min(size(distortedImageStacks,1),size(compedImageStacks,1)) - 3
                %Load files to compare
                compensatedScan = loadAsrlMatArchive([compDir compFilePath.name '/0001/' compedImageStacks(frameIndex).name]);                
                distortedScan  = loadAsrlMatArchive([distortedDir distortedFilePath.name '/0001/' distortedImageStacks(frameIndex).name]);
                
                %Get matching frame from reference series
                T = T_compframes_out(:,:,frameIndex);
                idx = FindReferenceFrame(T_ref_out,T);
                idx,frameIndex
                
                %Get reference image
                referenceFrame = loadAsrlMatArchive([referenceDir referenceImageStacks(idx).name]);
                
                [compScore(dirIndex,frameIndex) compTrackLegnth(dirIndex,frameIndex)] = CompareImagesByDescriptor(compensatedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'');
                [distortScore(dirIndex,frameIndex) distortTrackLegnth(dirIndex,frameIndex)] = CompareImagesByDescriptor(distortedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'');
                
                %Compare comped and uncomped scans
                if(DEBUG)
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
    
    figure(1); clf; hold on;
    plot((sum(compScore') ./ sum(compScore'>0)),'b');
    plot((sum(distortScore') ./ sum(distortScore'>0)),'r')
    legend('Compensated score','Distorted score');
    xlabel('Rotational speed');
    ylabel('Normalized matching score');
end