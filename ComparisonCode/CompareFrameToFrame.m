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
    
    figOutputPath = '~/Dropbox/Research/Images/Plots/'
    
    %Set root dirs from compensated and uncompensated image stacks
    %compDir = '/mnt/data/Datasets/Features/Compensated/02-Feb-13/ImageStacks/';
    %distortedDir = '/mnt/data/Datasets/Features/02-Feb-13/ImageStacks/';
    compDir = '/mnt/data/Datasets/Features/sim/compensated-data/'
    distortedDir = '/mnt/data/Datasets/Features/sim/distorted-data/'
    
    %Get directories containing image stacks
    %distortedDirs = dir([distortedDir 'smooth*']);
    %compDirs = dir([compDir 'smooth*']);
    distortedDirs = dir([distortedDir '*-*']);
    compDirs = dir([compDir '*-*']);
    
       
    %Loop over each directory and process file
    for dirIndex = 1 : size(distortedDirs,1)
        distortedFilePath = distortedDirs (dirIndex);
        compFilePath = compDirs(dirIndex);
        
        %Get image stacks in dir
        distortedImageStacks = dir([distortedDir distortedFilePath.name '/0001/*.asa']);
        compedImageStacks = dir([compDir compFilePath.name '/1.0/0001/*.asa']);
        
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
                compensatedScan1 = loadAsrlMatArchive([compDir compFilePath.name '/1.0/0001/' compedImageStacks(i).name]);
                compensatedScan2 = loadAsrlMatArchive([compDir compFilePath.name '/1.0/0001/' compedImageStacks(i+1).name]);
                distortedScan1  = loadAsrlMatArchive([distortedDir distortedFilePath.name '/0001/' distortedImageStacks(i).name]);
                distortedScan2  = loadAsrlMatArchive([distortedDir distortedFilePath.name '/0001/' distortedImageStacks(i+1).name]);
                
                if(DEBUG)
                    close all;
                end
                
                %compensatedScan1.intense8Img = compensatedScan1.intense8Img / brightnessFactor;
                %compensatedScan2.intense8Img = compensatedScan2.intense8Img / brightnessFactor;
                %distortedScan1.intense8Img = distortedScan1.intense8Img / brightnessFactor;
                %distortedScan2.intense8Img = distortedScan2.intense8Img / brightnessFactor;
                
                [SURFcompNormMatchingScore(dirIndex,i),SURFcompTrackLength(dirIndex,i)]       = CompareImagesByDescriptor(compensatedScan1.intense8Img,compensatedScan2.intense8Img,0,0,'SURF');
                [SURFdistortedNormMatchingScore(dirIndex,i), SURFdistortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan1.intense8Img,distortedScan2.intense8Img,0,0,'SURF');
                                
                [FASTcompNormMatchingScore(dirIndex,i),FASTcompTrackLength(dirIndex,i)]       = CompareImagesByDescriptor(compensatedScan1.intense8Img,compensatedScan2.intense8Img,0,0,'FAST');
                [FASTdistortedNormMatchingScore(dirIndex,i), FASTdistortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan1.intense8Img,distortedScan2.intense8Img,0,0,'FAST');
                
                [HarrisCompNormMatchingScore(dirIndex,i),HarrisCompTrackLength(dirIndex,i)]       = CompareImagesByDescriptor(compensatedScan1.intense8Img,compensatedScan2.intense8Img,0,0,'Harris');
                [HarrisDistortedNormMatchingScore(dirIndex,i), HarrisDistortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan1.intense8Img,distortedScan2.intense8Img,0,0,'Harris');
                
                [HarrAffCompNormMatchingScore(dirIndex,i),HarrAffCompTrackLength(dirIndex,i)]       = CompareImagesByDescriptor(compensatedScan1.intense8Img,compensatedScan2.intense8Img,0,0,'HarrisAffine');
                [HarrAffDistortedNormMatchingScore(dirIndex,i), HarrAffDistortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan1.intense8Img,distortedScan2.intense8Img,0,0,'HarrisAffine');
                 
                [MSERCompNormMatchingScore(dirIndex,i),MSERCompTrackLength(dirIndex,i)]       = CompareImagesByDescriptor(compensatedScan1.intense8Img,compensatedScan2.intense8Img,0,0,'MSER');
                [MSERDistortedNormMatchingScore(dirIndex,i), MSERDistortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan1.intense8Img,distortedScan2.intense8Img,0,0,'MSER');
                
                %Compute score
                compSumSURF    = compSumSURF + SURFcompNormMatchingScore(dirIndex,i);
                distortSumSURF = distortSumSURF + SURFdistortedNormMatchingScore(dirIndex,i); 
                
                compSumFAST    = compSumFAST + FASTcompNormMatchingScore(dirIndex,i);
                distortSumFAST = distortSumFAST + FASTdistortedNormMatchingScore(dirIndex,i); 
                
                compSumHarris    = compSumHarris + HarrisCompNormMatchingScore(dirIndex,i);
                distortSumHarris = distortSumHarris + HarrisDistortedNormMatchingScore(dirIndex,i); 
                
                compSumHarrAff    = compSumHarrAff + HarrAffCompNormMatchingScore(dirIndex,i);
                distortSumHarrAff = distortSumHarrAff + HarrAffDistortedNormMatchingScore(dirIndex,i); 
                
                compSumMSER    = compSumMSER + MSERCompNormMatchingScore(dirIndex,i);
                distortSumMSER = distortSumMSER + MSERDistortedNormMatchingScore(dirIndex,i); 
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
    uncompScoreSURF2 = [uncompScoreSURF(9) uncompScoreSURF(1:8) uncompScoreSURF(10:size(uncompScoreSURF,2))]  
    uncompStdSURF2 = [uncompStdSURF(9) uncompStdSURF(1:8) uncompStdSURF(10:size(uncompStdSURF,2))]
    
    uncompScoreFAST2 = [uncompScoreFAST(9) uncompScoreFAST(1:8) uncompScoreFAST(10:size(uncompScoreFAST,2))]
    uncompStdFAST2 = [uncompStdFAST(9) uncompStdFAST(1:8) uncompStdFAST(10:size(uncompStdFAST,2))]
    
    uncompScoreHarris2 = [uncompScoreHarris(9) uncompScoreHarris(1:8) uncompScoreHarris(10:size(uncompScoreHarris,2))]
    uncompStdHarris2 = [uncompStdHarris(9) uncompStdHarris(1:8) uncompStdHarris(10:size(uncompStdHarris,2))]
    
    uncompScoreHarraff2 = [uncompScoreHarrAff(9) uncompScoreHarrAff(1:8) uncompScoreHarrAff(10:size(uncompScoreHarrAff,2))]
    uncompStdHarraff2 = [uncompStdHarrAff(9) uncompStdHarrAff(1:8) uncompStdHarrAff(10:size(uncompStdHarrAff,2))]
    
    uncompScoreMSER2 = [uncompScoreMSER(9) uncompScoreMSER(1:8) uncompScoreMSER(10:size(uncompScoreMSER,2))]
    uncompStdMSER2 = [uncompStdMSER(9) uncompStdMSER(1:8) uncompStdMSER(10:size(uncompStdMSER,2))]
    
    %%%% COMP %%%
    compScoreSURF2 = [compScoreSURF(9) compScoreSURF(1:8) compScoreSURF(10:size(compScoreSURF,2))]
    compStdSURF2 = [compStdSURF(9) compStdSURF(1:8) compStdSURF(10:size(compStdSURF,2))]
    
    compScoreFAST2 = [compScoreFAST(9) compScoreFAST(1:8) compScoreFAST(10:size(compScoreFAST,2))]
    compStdFAST2 = [compStdFAST(9) compStdFAST(1:8) compStdFAST(10:size(compStdFAST,2))]
    
    compScoreHarris2 = [compScoreHarris(9) compScoreHarris(1:8) compScoreHarris(10:size(compScoreHarris,2))]
    compStdHarris2 = [compStdHarris(9) compStdHarris(1:8) compStdHarris(10:size(compStdHarris,2))]
    
    compScoreHarraff2 = [compScoreHarrAff(9) compScoreHarrAff(1:8) compScoreHarrAff(10:size(compScoreHarrAff,2))]
    compStdHarraff2 = [compStdHarrAff(9) compStdHarrAff(1:8) compStdHarrAff(10:size(compStdHarrAff,2))]
    
    compScoreMSER2 = [compScoreMSER(9) compScoreMSER(1:8) compScoreMSER(10:size(compScoreMSER,2))]
    compStdMSER2 = [compStdMSER(9) compStdMSER(1:8) compStdMSER(10:size(compStdMSER,2))]
     
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