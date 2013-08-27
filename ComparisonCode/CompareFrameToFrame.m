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
%   /name   CompareFrameToFrame
%   /brief  Compares image stacks by comparing sequential frames
%
function CompareFrameToFrame()
    DEBUG = 0;
    brightnessFactor = 256;
    
    %Set root dirs from compensated and uncompensated image stacks
    compDir = '/mnt/data/Datasets/Features/Compensated/02-Feb-13/ImageStacks/';
    distortedDir = '/mnt/data/Datasets/Features/02-Feb-13/ImageStacks/';
    
    %Get directories containing image stacks
    distortedDirs = dir([distortedDir 'smooth*']);
    compDirs = dir([compDir 'smooth*']);
       
    %Loop over each directory and process file
    for dirIndex = 1 : size(distortedDirs,1)
        distortedFilePath = distortedDirs (dirIndex);
        compFilePath = compDirs(dirIndex);
        
        %Get image stacks in dir
        distortedImageStacks = dir([distortedDir distortedFilePath.name '/0001/*.asa']);
        compedImageStacks = dir([compDir compFilePath.name '/0001/*.asa']);
        
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
        
        if(size(compedImageStacks,1)>0)
            for i = 1 : min(size(compedImageStacks,1),size(compedImageStacks,1)) - 3
                %Load files to compare
                compensatedScan1 = loadAsrlMatArchive([compDir compFilePath.name '/0001/' compedImageStacks(i).name]);
                compensatedScan2 = loadAsrlMatArchive([compDir compFilePath.name '/0001/' compedImageStacks(i+1).name]);
                distortedScan1  = loadAsrlMatArchive([distortedDir distortedFilePath.name '/0001/' distortedImageStacks(i).name]);
                distortedScan2  = loadAsrlMatArchive([distortedDir distortedFilePath.name '/0001/' distortedImageStacks(i+1).name]);
                
                if(DEBUG)
                    close all;
                end
                
                compensatedScan1.intense8Img = compensatedScan1.intense8Img / brightnessFactor;
                compensatedScan2.intense8Img = compensatedScan2.intense8Img / brightnessFactor;
                distortedScan1.intense8Img = distortedScan1.intense8Img / brightnessFactor;
                distortedScan2.intense8Img = distortedScan2.intense8Img / brightnessFactor;
                
                [SURFcompNormMatchingScore(dirIndex,i),SURFcompTrackLength(dirIndex,i)]       = CompareImagesByDescriptor(compensatedScan1.intense8Img,compensatedScan2.intense8Img,0,0,'SURF');
                [SURFdistortedNormMatchingScore(dirIndex,i), SURFdistortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan1.intense8Img,distortedScan2.intense8Img,0,0,'SURF');
                                
%                 [FASTcompNormMatchingScore(dirIndex,i),FASTcompTrackLength(dirIndex,i)]       = CompareImagesByDescriptor(compensatedScan1.intense8Img,compensatedScan2.intense8Img,0,0,'FAST');
%                 [FASTdistortedNormMatchingScore(dirIndex,i), FASTdistortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan1.intense8Img,distortedScan2.intense8Img,0,0,'FAST');
%                 
%                 [HarrisCompNormMatchingScore(dirIndex,i),HarrisCompTrackLength(dirIndex,i)]       = CompareImagesByDescriptor(compensatedScan1.intense8Img,compensatedScan2.intense8Img,0,0,'Harris');
%                 [HarrisDistortedNormMatchingScore(dirIndex,i), HarrisDistortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan1.intense8Img,distortedScan2.intense8Img,0,0,'Harris');
%                 
%                 [HarrAffCompNormMatchingScore(dirIndex,i),HarrAffCompTrackLength(dirIndex,i)]       = CompareImagesByDescriptor(compensatedScan1.intense8Img,compensatedScan2.intense8Img,0,0,'HarrisAffine');
%                 [HarrAffDistortedNormMatchingScore(dirIndex,i), HarrAffDistortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan1.intense8Img,distortedScan2.intense8Img,0,0,'HarrisAffine');
%                 
%                 [MSERCompNormMatchingScore(dirIndex,i),MSERCompTrackLength(dirIndex,i)]       = CompareImagesByDescriptor(compensatedScan1.intense8Img,compensatedScan2.intense8Img,0,0,'MSER');
%                 [MSERDistortedNormMatchingScore(dirIndex,i), MSERDistortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan1.intense8Img,distortedScan2.intense8Img,0,0,'MSER');
%                 
                %Compute score
                compSumSURF    = compSumSURF + SURFcompNormMatchingScore(dirIndex,i);
                distortSumSURF = distortSumSURF + SURFdistortedNormMatchingScore(dirIndex,i); 
                
%                 compSumFAST    = compSumFAST + FASTcompNormMatchingScore(dirIndex,i);
%                 distortSumFAST = distortSumFAST + FASTdistortedNormMatchingScore(dirIndex,i); 
%                 
%                 compSumHarris    = compSumHarris + HarrisCompNormMatchingScore(dirIndex,i);
%                 distortSumHarris = distortSumHarris + HarrisDistortedNormMatchingScore(dirIndex,i); 
%                 
%                 compSumHarrAff    = compSumHarrAff + HarrAffCompNormMatchingScore(dirIndex,i);
%                 distortSumHarrAff = distortSumHarrAff + HarrAffDistortedNormMatchingScore(dirIndex,i); 
%                 
%                 compSumMSER    = compSumMSER + MSERCompNormMatchingScore(dirIndex,i);
%                 distortSumMSER = distortSumMSER + MSERDistortedNormMatchingScore(dirIndex,i); 
            end
            
            if(DEBUG)
                figure(1); clf; hold on;
                plot(distortedNormMatchingScore(dirIndex,:),'r'); 
                plot(compNormMatchingScore(dirIndex,:),'b'); 
                hold off;

                sum(distortedNormMatchingScore(dirIndex,:)), sum(compNormMatchingScore(dirIndex,:))
            end
        end

        compScoreSURF(dirIndex) = compSumSURF / size(compedImageStacks,1);
        string = sprintf('Average normalized matching score for compensated images: %f',compScoreSURF(dirIndex));
        disp(string);

        uncompScoreSURF(dirIndex) = distortSumSURF / size(compedImageStacks,1);
        string = sprintf('Average normalized matching score for distorted images: %f',uncompScoreSURF(dirIndex));
        disp(string);
        
%         compScoreFAST(dirIndex) = compSumFAST/ size(compedImageStacks,1);
%         string = sprintf('Average normalized matching score for compensated images: %f',compScoreFAST(dirIndex));
%         disp(string);
% 
%         uncompScoreFAST(dirIndex) = distortSumFAST / size(compedImageStacks,1);
%         string = sprintf('Average normalized matching score for distorted images: %f',uncompScoreFAST(dirIndex));
%         disp(string);
%         
%         compScoreHarris(dirIndex) = compSumHarris / size(compedImageStacks,1);
%         string = sprintf('Average normalized matching score for compensated images: %f',compScoreHarris(dirIndex));
%         disp(string);
% 
%         uncompScoreHarris(dirIndex) = distortSumHarris / size(compedImageStacks,1);
%         string = sprintf('Average normalized matching score for distorted images: %f',uncompScoreHarris(dirIndex));
%         disp(string);
%         
%         compScoreHarrAff(dirIndex) = compSumHarrAff / size(compedImageStacks,1);
%         string = sprintf('Average normalized matching score for compensated images: %f',compScoreHarrAff(dirIndex));
%         disp(string);
% 
%         uncompScoreHarrAff(dirIndex) = distortSumHarrAff / size(compedImageStacks,1);
%         string = sprintf('Average normalized matching score for distorted images: %f',uncompScoreHarrAff(dirIndex));
%         disp(string);
%         
%         compScoreMSER(dirIndex) = compSumMSER / size(compedImageStacks,1);
%         string = sprintf('Average normalized matching score for compensated images: %f',compScoreMSER(dirIndex));
%         disp(string);
% 
%         uncompScoreMSER(dirIndex) = distortSumMSER / size(compedImageStacks,1);
%         string = sprintf('Average normalized matching score for distorted images: %f',uncompScoreMSER(dirIndex));
%         disp(string);
    end
    
    
    %%%%%%%%%%%%%%%%
    % Plot results %
    %%%%%%%%%%%%%%%%
    
    figure(1); clf; hold on;
    plot(compScore,'b','LineWidth',2);
    plot(uncompScore,'r','LineWidth',2);
    xlabel('Rotational velocity');
    ylabel('Average normalized matching score');  
    legend('Features matching on comp. images','Features matching on uncomp. images');
end