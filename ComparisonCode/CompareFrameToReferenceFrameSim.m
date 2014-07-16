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
function CompareFrameToReferenceFrameSim()
    kFrameTime = 0.5; %2 Hz lidarg
    fov = [90 45];
    DEBUG = 0;
    brightnessFactor = 1;
       
    figOutputPath = '~/Dropbox/Research/Images/Plots/'
    
    %Set root dirs from compensated and uncompensated image stacks
    compRootDir = '/run/media/erik/data/Datasets/Features/sim/mars/compensated-data/'
    distortedRootDir = '/run/media/erik/data/Datasets/Features/sim/mars/distorted-data/';
    
    %Set dirs from which to load imagestack   
    compDir = [compRootDir ''];
    distortedDir = [distortedRootDir ''];
    
    %Get directories containing image stacks
    distortedDirs = dir([distortedDir '*-*-*']);
    compDirs = dir([compDir '*-*-*']);
    referenceDir = [distortedDir '/undistorted/0001/']
    
    %Load reference image stack directory listing
    files = dir([referenceDir '*.asa']);
    for i = 1 : size(files,1)
        referenceImageStacks(i).name = [num2str(i) '.asa'];
    end
    
    %Loop over each directory and process file
    for dirIndex = 1 : size(distortedDirs,1)
        distortedFilePath = distortedDirs (dirIndex);
        compFilePath = compDirs(dirIndex);
        
        %Get image stacks in dir
        distortedImageStacks = dir([distortedDir distortedFilePath.name '/0001/*.asa']);
        compedImageStacks = dir([compDir compFilePath.name '/1.0/0001/*.asa']);
                
        %Get speed from the filename
        speed = str2num(cell2mat(regexp(distortedFilePath.name,'^\d+','match')))
        
        compSumSURF = 0;
        distortSumSURF = 0;
        
        compSumFAST = 0;
    
        
        compSum = 0;
        distortSum = 0;
        if(size(compedImageStacks,1)>0)
            for frameIndex = 1 : min([size(distortedImageStacks,1),size(compedImageStacks,1)]) - 3
                %Load files to compare
                compensatedScan = loadAsrlMatArchive([compDir compFilePath.name '/1.0/0001/' compedImageStacks(frameIndex).name]);                
                distortedScan  = loadAsrlMatArchive([distortedDir distortedFilePath.name '/0001/' distortedImageStacks(frameIndex).name]);
                
                %Compute 
                refIdx = round(mod(speed * kFrameTime * frameIndex,359));
                
                %Get reference image
                try
                    referenceFrame = loadAsrlMatArchive([referenceDir referenceImageStacks(refIdx).name]);
                catch
                    disp('error!')
                end
                close all;
                
                %Run comparison for different detectors with the same
                %descriptor (SURF)
                [SURFcompNormMatchingScore(dirIndex,frameIndex), SURFcompTrackLength(dirIndex,frameIndex)] = CompareImagesByDescriptor(compensatedScan,referenceFrame,0,0,'SURF');
                pause(2);
                [SURFdistortedNormMatchingScore(dirIndex,frameIndex), SURFdistortTrackLength(dirIndex,frameIndex)] = CompareImagesByDescriptor(distortedScan,referenceFrame,0,0,'SURF');                
                pause(2);
            end
        end        
        
        meanCompScoreSURF(dirIndex) = mean(SURFcompNormMatchingScore(dirIndex,SURFcompNormMatchingScore(dirIndex,:) > 0));
        stdCompSURF(dirIndex) = std(SURFcompNormMatchingScore(dirIndex,SURFcompNormMatchingScore(dirIndex,:)>0));
        meanUncompScoreSURF(dirIndex) = mean(SURFdistortedNormMatchingScore(dirIndex,SURFdistortedNormMatchingScore(dirIndex,:) > 0));
        stdUncompSURF(dirIndex) = std(SURFdistortedNormMatchingScore(dirIndex,SURFdistortedNormMatchingScore(dirIndex,:)>0));
    end
    
    %%%%%%%%%%%%%%%%
% Plot results %
%%%%%%%%%%%%%%%%

%%% UNCOMP %%%k
rotVel = [10,15,20,25,30,35,40,45,50,60,90];
%uncompScoreSURF = [meanUncompScoreSURF(9) meanUncompScoreSURF(1:8) meanUncompScoreSURF(10:size(meanUncompScoreSURF,2))]
%uncompStdSURF = [stdUncompSURF(9) stdUncompSURF(1:8) stdUncompSURF(10:size(stdUncompSURF,2))]

%%%% COMP %%%
%compScoreSURF = [meanCompScoreSURF(9) meanCompScoreSURF(1:8) meanCompScoreSURF(10:size(meanCompScoreSURF,2))]
%compStdSURF = [stdCompSURF(9) stdCompSURF(1:8) stdCompSURF(10:size(stdCompSURF,2))]

h = figure(1); clf; hold on;
plot(rotVel,meanUncompScoreSURF,'b','LineWidth',2);
plot(rotVel,meanCompScoreSURF,'r','LineWidth',2);
legend('Features matching on uncomp. images','Features matching on comp. images');
plot(rotVel,meanUncompScoreSURF,'xb','LineWidth',2);
plot(rotVel,meanCompScoreSURF,'xr','LineWidth',2);
xlabel('Rotational velocity (deg/s)');
ylabel('Average normalized matching score');  
title('Performance of SURF vs. rotational velocity')
errorbar(rotVel,meanCompScoreSURF,stdCompSURF,'r');
errorbar(rotVel,meanUncompScoreSURF,stdUncompSURF,'b');
print(h,[figOutputPath 'perf-vs-speed-surf'],'-depsc'); 

end



 