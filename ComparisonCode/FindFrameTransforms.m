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
%   /name   AssociateTransformsWithFrames
%   /brief  Compares each frame from a particular trial to a frame from a
%           reference set of image stacks.
function [out_frameTransforms] = FindFrameTransforms(in_imagestackDir,in_T,in_t)
    DEBUG = 0;
    kNumDataPoints = 5 
    kMoveThreshold = 0.05 % meters
    
    % Find the point were the ground truth transformations start changing
    % i.e., robot starts motion.
    refViconOffset = 1;
    startPosition = mean(in_T(1:4,4,1:kNumDataPoints),3);
    for transformIndex = 1 : (size(in_T,3) - kNumDataPoints)
        meanTranslation = mean(in_T(1:4,4,transformIndex:transformIndex+kNumDataPoints),3);
        
        if(sum(abs(meanTranslation - startPosition)) > kMoveThreshold)
            refViconOffset = transformIndex + 100;
            break;
        end
    end
        
    % Get image stacks in the reference directory
    files = dir([in_imagestackDir '*.asa'])    
    
    % Load the first frame (so we can use the timestamps as start time)
    firstFrame =  loadAsrlMatArchive([in_imagestackDir files(1).name]);
    
    % Loop over all files, and find the index of the associated transform
    % by matching timestamps
    frameIndex = 1;
    for file = files'
        currentFrame = loadAsrlMatArchive([in_imagestackDir file.name]);        
        
        %Get transform index
        transformIdx = refViconOffset + MatchTimestamps(currentFrame .timestamp,firstFrame.timestamp,in_t(1,refViconOffset:size(in_t,2)));                        
        
        %Get the transformation matrix at this index
        out_frameTransforms(:,:,frameIndex) = in_T(:,:,transformIdx);
        
        frameIndex = frameIndex + 1;
    end
    
    if(DEBUG)
        figure; clf;
        hold on; axis equal; grid on;
        xlabel('x'); ylabel('y'); zlabel('z')
        plotCoordinateFrame(rph2c([pi/2,-pi/2,pi]),[0,0,0]')

        %Plot each frame
        for i = 1:size(T_ref_out,3)
            T_t = T_ref_out(:,:,i);
            plotCoordinateFrame(T_t(1:3,1:3),T_t(1:3,4),2); 
        end
    end
    
end