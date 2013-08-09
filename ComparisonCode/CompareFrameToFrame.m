function CheckImagesRealData()
    DEBUG = 0;

    %Set root dirs from compensated and uncompensated image stacks
    compDir = '/mnt/data/Datasets/Features/RANSAC-Comp/ImageStacks/';
    distortedDir = '/mnt/data/Datasets/Features/02-Feb-13/ImageStacks/';
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Match each frame to the next %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Get dirs
    distortedDirs = dir([distortedDir 'smooth*']);
    compDirs = dir([compDir 'smooth*']);
       
    for dirIndex = 2 : size(distortedDirs,1)
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
                compensatedScan1 = loadAsrlMatArchive([compDir compFilePath.name '/0001/' compedImageStacks(i).name]);
                compensatedScan2 = loadAsrlMatArchive([compDir compFilePath.name '/0001/' compedImageStacks(i+1).name]);
                distortedScan1  = loadAsrlMatArchive([distortedDir distortedFilePath.name '/0001/' distortedImageStacks(i).name]);
                distortedScan2  = loadAsrlMatArchive([distortedDir distortedFilePath.name '/0001/' distortedImageStacks(i+1).name]);

                
%                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 %%%%% Common Image Parts %%%%%%%
%                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 %Compute commmon image parts from motion                                      
%                 movementInPixels = round((size(compensatedScan1.intense8Img,2)/fov(1)) * vpChange);
%                 commonImgPart1.intense8Img = compensatedScan1.intense8Img(:,1:((size(compensatedScan1.intense8Img,2)+1) - movementInPixels));
%                 commonImgPart2.intense8Img = compensatedScan2.intense8Img(:,movementInPixels:size(compensatedScan1.intense8Img,2));
%                 commonImgPart1.maskImg = compensatedScan1.maskImg(:,1:((size(compensatedScan1.intense8Img,2)+1) - movementInPixels));
%                 commonImgPart2.maskImg = compensatedScan2.maskImg(:,movementInPixels:size(compensatedScan1.intense8Img,2));
% 
%                 %Crop blackness due to compensation
%                 commonImgPart1.intense8Img = commonImgPart1.intense8Img(:,sum(commonImgPart1.maskImg) == size(commonImgPart1.maskImg,1))
%                 commonImgPart2.intense8Img = commonImgPart2.intense8Img(:,sum(commonImgPart2.maskImg) == size(commonImgPart2.maskImg,1))
%                 commonImgPart1.maskImg = commonImgPart1.maskImg(:,sum(commonImgPart1.maskImg) == size(commonImgPart1.maskImg,1))
%                 commonImgPart2.maskImg = commonImgPart2.maskImg(:,sum(commonImgPart2.maskImg) == size(commonImgPart2.maskImg,1))
% 
%                 %Check if the images are of equal size, if not, make
%                 %them equal
%                 newSize = min(size(commonImgPart1.intense8Img,2),size(commonImgPart2.intense8Img,2))
%                 commonImgPart1.intense8Img = commonImgPart1.intense8Img(:,1:newSize);
%                 commonImgPart2.intense8Img = commonImgPart2.intense8Img(:,1:newSize);
                
                close all;
                fileNamePostFix = sprintf('real-lowthres-compensated-%d',i);
                compensatedScan1.intense8Img = compensatedScan1.intense8Img  / 256;
                compensatedScan2.intense8Img = compensatedScan2.intense8Img  / 256;
                [compNormMatchingScore(dirIndex,i),compTrackLength(dirIndex,i)]       = CompareImagesByDescriptor(compensatedScan1.intense8Img, compensatedScan2.intense8Img,DEBUG,0,fileNamePostFix);
                
                fileNamePostFix = sprintf('real-lowthres-uncomp-%d',i);
                distortedScan1.intense8Img = distortedScan1.intense8Img  / 256;
                distortedScan2.intense8Img = distortedScan2.intense8Img  / 256;
                [distortedNormMatchingScore(dirIndex,i),distortTrackLength(dirIndex,i)] = CompareImagesByDescriptor(distortedScan1.intense8Img, distortedScan2.intense8Img,DEBUG,0,fileNamePostFix);
                
                %Compute score
                compSum    = compSum + compNormMatchingScore(dirIndex,i);
                distortSum = distortSum + distortedNormMatchingScore(dirIndex,i); 
            end
            
            figure(1); clf; hold on;
            plot(distortedNormMatchingScore(dirIndex,:),'r'); 
            plot(compNormMatchingScore(dirIndex,:),'b'); 
            hold off;
            
            sum(distortedNormMatchingScore(dirIndex,:)), sum(compNormMatchingScore(dirIndex,:))
        end

        distortedFilePath.name
        compFilePath.name
        
        compScore(dirIndex) = compSum / size(compedImageStacks,1);
        string = sprintf('Average normalized matching score for compensated images: %f',compScore(dirIndex));
        disp(string);

        uncompScore(dirIndex) = distortSum / size(compedImageStacks,1);
        string = sprintf('Average normalized matching score for distorted images: %f',uncompScore(dirIndex));
        disp(string);
    end
    
    figure(1); clf; hold on;
    plot(compScore,'b','LineWidth',2);
    plot(uncompScore,'r','LineWidth',2);
    xlabel('Rotational velocity');
    ylabel('Average normalized matching score');  
    legend('Features matching on comp. images','Features matching on uncomp. images');
    
    %Average track length
    %sum(compTrackLength(2,~isnan(compTrackLength(2,:)))) / sum(~isnan(compTrackLength(2,:)))
    %sum(distortTrackLength(2,~isnan(distortTrackLength(2,:)))) / sum(~isnan(distortTrackLength(2,:)))
end