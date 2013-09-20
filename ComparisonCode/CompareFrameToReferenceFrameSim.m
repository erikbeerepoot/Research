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

    
    kFrameTime = 0.5; %2 Hz lidar
    DEBUG = 0;
    brightnessFactor = 1;
    autoComputeViewpoint = 0;
    
    %Set root dirs from compensated and uncompensated image stacks
    compRootDir = '/mnt/data/Datasets/Features/sim/compensated-data/'
    distortedRootDir = '/mnt/data/Datasets/Features/sim/distorted-data/';
    
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
        distortSumFAST = 0;
        
        compSumHarris = 0;
        distortSumHarris = 0;
        
        compSumHarrAff = 0;
        distortSumHarrAff = 0;
        
        compSumMSER = 0;
        distortSumMSER = 0;
        
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
                    disp('balls!')
                end
                close all;
                
                %Run comparison for different detectors with the same
                %descriptor (SURF)
                [SURFcompNormMatchingScore(dirIndex,frameIndex) SURFcompTrackLength(dirIndex,frameIndex)] = CompareImagesByDescriptor(compensatedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'SURF');
                [SURFdistortedNormMatchingScore(dirIndex,frameIndex) SURFdistortTrackLength(dirIndex,frameIndex)] = CompareImagesByDescriptor(distortedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'SURF');
                
                [FASTcompNormMatchingScore(dirIndex,i),FASTcompTrackLength(dirIndex,i)]  = CompareImagesByDescriptor(compensatedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'FAST');
                [FASTdistortedNormMatchingScore(dirIndex,i), FASTdistortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'FAST');
                
                [HarrisCompNormMatchingScore(dirIndex,i),HarrisCompTrackLength(dirIndex,i)] = CompareImagesByDescriptor(compensatedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'Harris');
                [HarrisDistortedNormMatchingScore(dirIndex,i), HarrisDistortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'Harris');
                
                [HarrAffCompNormMatchingScore(dirIndex,i),HarrAffCompTrackLength(dirIndex,i)] = CompareImagesByDescriptor(compensatedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'HarrisAffine');
                [HarrAffDistortedNormMatchingScore(dirIndex,i), HarrAffDistortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'HarrisAffine');
                
                [MSERCompNormMatchingScore(dirIndex,i),MSERCompTrackLength(dirIndex,i)] = CompareImagesByDescriptor(compensatedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'MSER');
                [MSERDistortedNormMatchingScore(dirIndex,i), MSERDistortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan.intense8Img/brightnessFactor,referenceFrame.intense8Img/brightnessFactor,0,0,'MSER');

                %Compute score
                compSumSURF    = compSumSURF + SURFcompNormMatchingScore(dirIndex,frameIndex);
                distortSumSURF = distortSumSURF + SURFdistortedNormMatchingScore(dirIndex,frameIndex); 
                
                compSumFAST    = compSumFAST + FASTcompNormMatchingScore(dirIndex,i);
                distortSumFAST = distortSumFAST + FASTdistortedNormMatchingScore(dirIndex,i); 
                
                compSumHarris    = compSumHarris + HarrisCompNormMatchingScore(dirIndex,i);
                distortSumHarris = distortSumHarris + HarrisDistortedNormMatchingScore(dirIndex,i); 
                
                compSumHarrAff    = compSumHarrAff + HarrAffCompNormMatchingScore(dirIndex,i);
                distortSumHarrAff = distortSumHarrAff + HarrAffDistortedNormMatchingScore(dirIndex,i); 
                
                compSumMSER    = compSumMSER + MSERCompNormMatchingScore(dirIndex,i);
                distortSumMSER = distortSumMSER + MSERDistortedNormMatchingScore(dirIndex,i); 
                
            end
        end
        compScoreSURF(dirIndex) = compSumSURF / size(compedImageStacks,1);
        compStdSURF(dirIndex) = std(SURFcompNormMatchingScore(dirIndex,SURFcompNormMatchingScore(dirIndex,:)>0))
        uncompScoreSURF(dirIndex) = distortSumSURF / size(compedImageStacks,1);
        uncompStdSURF(dirIndex) = std(SURFdistortedNormMatchingScore(dirIndex,SURFdistortedNormMatchingScore(dirIndex,:)>0))
        
        compScoreFAST(dirIndex) = compSumFAST/ size(compedImageStacks,1);
        compStdFAST(dirIndex) = std(FASTcompNormMatchingScore(dirIndex,FASTcompNormMatchingScore(dirIndex,:)>0))
        uncompScoreFAST(dirIndex) = distortSumFAST / size(compedImageStacks,1);
        uncompStdFAST(dirIndex) = std(FASTdistortedNormMatchingScore(dirIndex,FASTdistortedNormMatchingScore(dirIndex,:)>0))
        
        compScoreHarris(dirIndex) = compSumHarris / size(compedImageStacks,1);
        compStdHarris(dirIndex) = std(HarrisCompNormMatchingScore(dirIndex,HarrisCompNormMatchingScore(dirIndex,:)>0))
        uncompScoreHarris(dirIndex) = distortSumHarris / size(compedImageStacks,1);
        uncompStdHarris(dirIndex) = std(HarrisCompNormMatchingScore(dirIndex,HarrisCompNormMatchingScore(dirIndex,:)>0))
        
        compScoreHarrAff(dirIndex) = compSumHarrAff / size(compedImageStacks,1);
        compStdHarrAff(dirIndex) = std(HarrAffCompNormMatchingScore(dirIndex,HarrAffCompNormMatchingScore(dirIndex,:)>0));
        uncompScoreHarrAff(dirIndex) = distortSumHarrAff / size(compedImageStacks,1);
        uncompStdHarrAff(dirIndex) = std(HarrAffDistortedNormMatchingScore(dirIndex,HarrAffDistortedNormMatchingScore(dirIndex,:)>0));
         
        compScoreMSER(dirIndex) = compSumMSER / size(compedImageStacks,1);
        compStdMSER(dirIndex) = std(MSERCompNormMatchingScore(dirIndex,MSERCompNormMatchingScore(dirIndex,:)>0));
        uncompScoreMSER(dirIndex) = distortSumMSER / size(compedImageStacks,1);
        uncompStdMSER(dirIndex) = std(MSERDistortedNormMatchingScore(dirIndex,MSERDistortedNormMatchingScore(dirIndex,:)>0));
 
    end
    
    %%%%%%%%%%%%%%%%
% Plot results %
%%%%%%%%%%%%%%%%

%%% UNCOMP %%%
rotVel = [5,10,15,20,25,30,35,40,45,50,55,60,90];
h = figure(1); clf; hold on;
plot(rotVel,compScoreSURF2,'b','LineWidth',2);
plot(rotVel,uncompScoreSURF2,'r','LineWidth',2);
legend('Features matching on comp. images','Features matching on uncomp. images');
plot(rotVel,compScoreSURF2,'xb','LineWidth',2);
plot(rotVel,uncompScoreSURF2,'xr','LineWidth',2);
xlabel('Rotational velocity (deg/s)');
ylabel('Average normalized matching score');  
title('Performance of SURF vs. rotational velocity')
errorbar(rotVel,uncompScoreSURF2,compStdSURF2,'r');
errorbar(rotVel,compScoreSURF2,uncompStdSURF,'b');
print(h,[figOutputPath 'perf-vs-speed-surf'],'-depsc'); 

h = figure(2); clf; hold on;
plot(rotVel,compScoreFAST2,'b','LineWidth',2);
plot(rotVel,uncompScoreFAST2,'r','LineWidth',2);
legend('Features matching on comp. images','Features matching on uncomp. images');
plot(rotVel,compScoreFAST2,'xb','LineWidth',2);
plot(rotVel,uncompScoreFAST2,'xr','LineWidth',2);
xlabel('Rotational velocity');
ylabel('Average normalized matching score');  
title('Performance of FAST vs. rotational velocity')
errorbar(rotVel,uncompScoreFAST2,uncompStdFAST2,'r');
errorbar(rotVel,compScoreFAST2,compStdFAST2,'b');
print(h,[figOutputPath 'perf-vs-speed-fast'],'-depsc'); 

h = figure(3); clf; hold on;
plot(rotVel,compScoreHarris2,'b','LineWidth',2);
plot(rotVel,uncompScoreHarris2,'r','LineWidth',2);
legend('Features matching on comp. images','Features matching on uncomp. images');
plot(rotVel,compScoreHarris2,'xb','LineWidth',2);
plot(rotVel,uncompScoreHarris2,'xr','LineWidth',2);
xlabel('Rotational velocity');
ylabel('Average normalized matching score');
title('Performance of Harris Corners vs. rotational velocity')
errorbar(rotVel,uncompScoreHarris2,uncompStdHarris2,'r');
errorbar(rotVel,compScoreHarris2,compStdHarris2,'b');
print(h,[figOutputPath 'perf-vs-speed-harris'],'-depsc'); 

h = figure(4); clf; hold on;
plot(rotVel,compScoreMSER2,'b','LineWidth',2);
plot(rotVel,uncompScoreMSER2,'r','LineWidth',2);
legend('Features matching on comp. images','Features matching on uncomp. images');
plot(rotVel,compScoreMSER2,'xb','LineWidth',2);
plot(rotVel,uncompScoreMSER2,'xr','LineWidth',2);
xlabel('Rotational velocity');
ylabel('Average normalized matching score');  
title('Performance of MSER vs. rotational velocity')
errorbar(rotVel,uncompScoreMSER2,uncompStdMSER2,'r');
errorbar(rotVel,compScoreMSER2,compStdMSER2,'b');
print(h,[figOutputPath 'perf-vs-speed-mser'],'-depsc'); 

h = figure(5); clf; hold on;
plot(rotVel,compScoreHarraff2,'b','LineWidth',2);
plot(rotVel,uncompScoreHarraff2,'r','LineWidth',2);
legend('Features matching on comp. images','Features matching on uncomp. images');
plot(rotVel,compScoreHarraff2,'xb','LineWidth',2);
plot(rotVel,uncompScoreHarraff2,'xr','LineWidth',2);
xlabel('Rotational velocity');
ylabel('Average normalized matching score');  
title('Performance of Harris-Affine vs. rotational velocity')
errorbar(rotVel,uncompScoreHarraff2,uncompStdHarraff2,'r');
errorbar(rotVel,compScoreHarraff2,compStdHarraff2,'b');
print(h,[figOutputPath 'perf-vs-speed-haraff'],'-depsc'); 

% Uncomp all features
h = figure(6); clf; hold on;
plot(rotVel,uncompScoreSURF2,'b','LineWidth',2);
plot(rotVel,uncompScoreFAST2,'r','LineWidth',2);
plot(rotVel,uncompScoreHarris2,'g','LineWidth',2);
plot(rotVel,uncompScoreHarraff2,'k','LineWidth',2);
plot(rotVel,uncompScoreMSER2,'c','LineWidth',2);
legend('SURF','FAST','Harris Corners','Harris-Affine','MSER')
title('Normalized matching score vs. rotational velocity on uncompensated images');
plot(rotVel,uncompScoreSURF2,'bx','LineWidth',2);
plot(rotVel,uncompScoreFAST2,'rx','LineWidth',2);
plot(rotVel,uncompScoreHarris2,'gx','LineWidth',2);
plot(rotVel,uncompScoreHarraff2,'kx','LineWidth',2);
plot(rotVel,uncompScoreMSER2,'cx','LineWidth',2);
errorbar(rotVel,uncompScoreSURF2,compStdSURF2,'b');
errorbar(rotVel,uncompScoreMSER2,uncompStdMSER2,'c');
errorbar(rotVel,uncompScoreHarris2,uncompStdHarris2,'g');
errorbar(rotVel,uncompScoreHarraff2,uncompStdHarraff2,'k');
errorbar(rotVel,uncompScoreFAST2,uncompStdFAST2,'r');  
print(h,[figOutputPath 'perf-vs-speed-uncomp'],'-depsc'); 

% comp all features
h = figure(7); clf; hold on;
plot(rotVel,compScoreSURF2,'b','LineWidth',2);
plot(rotVel,compScoreFAST2,'r','LineWidth',2);
plot(rotVel,compScoreHarris2,'g','LineWidth',2);
plot(rotVel,compScoreHarraff2,'k','LineWidth',2);
plot(rotVel,compScoreMSER2,'c','LineWidth',2);
legend('SURF','FAST','Harris Corners','Harris-Affine','MSER')
title('Normalized matching score vs. rotational velocity on compensated images');
plot(rotVel,compScoreSURF2,'bx','LineWidth',2);
plot(rotVel,compScoreFAST2,'rx','LineWidth',2);
plot(rotVel,compScoreHarris2,'gx','LineWidth',2);
plot(rotVel,compScoreHarraff2,'kx','LineWidth',2);
plot(rotVel,compScoreMSER2,'cx','LineWidth',2);
errorbar(rotVel,compScoreMSER2,compStdMSER2,'c');
errorbar(rotVel,compScoreHarris2,compStdHarris2,'g');
errorbar(rotVel,compScoreHarraff2,compStdHarraff2,'k');
errorbar(rotVel,compScoreFAST2,compStdFAST2,'r');
errorbar(rotVel,compScoreSURF2,uncompStdSURF,'b');
print(h,[figOutputPath 'perf-vs-speed-comp'],'-depsc');   

end



 