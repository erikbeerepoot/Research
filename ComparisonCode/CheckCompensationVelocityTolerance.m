function CheckCompensationVelocityTolerance()
    
    %compDir = '~/Dropbox/ASRL/sim/compensated-data/';
    compDir = '/mnt/data/Datasets/Features/sim/compensated-data/'
    fov = [90 45];
    vpChange = 30;    
    DEBUG = 0;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Match each frame to the next %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Get dirs
    simSpeedsDirs = dir(compDir);
    
    % Exclude '.', '..', and mac resource forks.
    index = 1;
    for dirIdx = 1: length(simSpeedsDirs)
        if(simSpeedsDirs(dirIdx).name(1)~='.' && simSpeedsDirs(dirIdx).isdir==1)
            simSpeedsDirlist(index) = simSpeedsDirs(dirIdx);
            index = index + 1;
        end
    end
    
    simSpeedsDirlist = SortFilesWithNumber(simSpeedsDirlist)
    
            
    for dirIndex = 1 : size(simSpeedsDirlist,2)                               
        %Get dirs
        diffCompSpeedDirs = dir([compDir simSpeedsDirlist(dirIndex).name]);

        % Exclude '.', '..', and mac resource forks.
        index = 1;
        for dirIdx = 1: length(diffCompSpeedDirs)
            if(diffCompSpeedDirs(dirIdx).name(1)~='.' && diffCompSpeedDirs(dirIdx).isdir==1)
                compSpeedsDirlist(index) = diffCompSpeedDirs(dirIdx);
                index = index + 1;
            end
        end
        
        
        for compSpeedIndex = 1 : size(compSpeedsDirlist,2)
            currentDir = compSpeedsDirlist(compSpeedIndex);
            
            files = dir([compDir simSpeedsDirlist(dirIndex).name '/' currentDir.name '/0001/*.asa'])
            simSpeedsDirlist(dirIndex).name 
            compSum = 0;
            
            if(size(files,1)>0)
                for i = 1 : min(size(files,1)) - 1
                    %Load files to compare
                    compensatedScan1 = loadAsrlMatArchive([compDir simSpeedsDirlist(dirIndex).name '/' currentDir.name '/0001/' files(i).name]);
                    compensatedScan2 = loadAsrlMatArchive([compDir simSpeedsDirlist(dirIndex).name '/' currentDir.name '/0001/' files(i+1).name]);

                    trackLengthThres = inf;
                    
                    %Compute commmon image parts from motion
%                     vpChange = str2num(cell2mat(regexp(simSpeedsDirlist(dirIndex).name,'^[0-9]+','match')));
%                     movementInPixels = round((size(compensatedScan1.intense8Img,2)/fov(1)) * vpChange);
%                     commonImgPart1.intense8Img = compensatedScan1.intense8Img(:,1:((size(compensatedScan1.intense8Img,2)+1) - movementInPixels));
%                     commonImgPart2.intense8Img = compensatedScan2.intense8Img(:,movementInPixels:size(compensatedScan1.intense8Img,2));
%                     commonImgPart1.maskImg = compensatedScan1.maskImg(:,1:((size(compensatedScan1.intense8Img,2)+1) - movementInPixels));
%                     commonImgPart2.maskImg = compensatedScan2.maskImg(:,movementInPixels:size(compensatedScan1.intense8Img,2));
                         
                    
                    commonImgPart1.intense8Img = compensatedScan1.intense8Img;
                    commonImgPart2.intense8Img = compensatedScan2.intense8Img;
                    commonImgPart1.maskImg = compensatedScan1.maskImg;
                    commonImgPart2.maskImg = compensatedScan2.maskImg;
                    
                    
                    %Crop blackness due to compensation
%                     commonImgPart1.intense8Img = commonImgPart1.intense8Img(:,sum(commonImgPart1.maskImg) > (size(commonImgPart1.maskImg,1)-5))
%                     commonImgPart2.intense8Img = commonImgPart2.intense8Img(:,sum(commonImgPart2.maskImg) > (size(commonImgPart2.maskImg,1)-5))
%                     commonImgPart1.maskImg = commonImgPart1.maskImg(:,sum(commonImgPart1.maskImg) > (size(commonImgPart1.maskImg,1)-5))
%                     commonImgPart2.maskImg = commonImgPart2.maskImg(:,sum(commonImgPart2.maskImg) > (size(commonImgPart2.maskImg,1)-5))
                    
                    %Check if the images are of equal size, if not, make
                    %them equal
%                     newSize = min(size(commonImgPart1.intense8Img,2),size(commonImgPart2.intense8Img,2))
%                     commonImgPart1.intense8Img = commonImgPart1.intense8Img(:,1:newSize);
%                     commonImgPart2.intense8Img = commonImgPart2.intense8Img(:,1:newSize);
                    
                    
                    
                    fileNamePostFix = sprintf('real-lowthres-compensated-%d',i);                    
                    [compNormMatchingScore,compTrackLength(dirIndex,i)]       = CompareImagesByDescriptor(commonImgPart1.intense8Img,commonImgPart2.intense8Img,DEBUG,0,'SURF');
                                            
                    %Compute score
                    compSum = compSum + compNormMatchingScore;
                end
            end
            
            % -1 since we're comparing n-1 pairs
            compScore(dirIndex,compSpeedIndex) = compSum / (size(files,1)-1);
            
            if(DEBUG)
                string = sprintf('Average normalized matching score for compensated images: %f',compScore(dirIndex));
                disp(string);
            end
        end        
    end
    
    
    %%%% PLOT RESULTS %%%%
    for index = 1 : size(compSpeedsDirlist,2)
        x(index) = str2num(compSpeedsDirlist(index).name);
    end
    
     figure(1); clf; hold on;
     cc = hsv(size(compScore,1));
     for index = 1 : size(compScore,1)
         plot(x,compScore(index,:),'Color',cc(index,:),'LineWidth',2,'Marker','x');
     end
     
     xlabel('Fraction of groundtruth velocity used for compensation');
     ylabel('Average normalized matching score');  
     legend(simSpeedsDirlist.name);

   
    
end


function sortedFiles = SortFilesWithNumber(files)

    %Must follow format ##-deg-s (filter the files)
    k=1
    for i = 1 : size(files,2)
        [name,~] = regexp(files(i).name,'^[0-9]+-[a-z]+-[a-z]$','match')
        if(~isempty(name))
            outfiles(k) = files(i);
            k = k + 1;
        end
    end
    
    %Parse numbers out of the dirname
    for i = 1 : size(outfiles,2)
        nums(i) = str2num(cell2mat(regexp(files(i).name,'^[0-9]+','match')));
    end
    
    %Get indices to use to sort the other array
    [sorted,I] = sort(nums);
    
    %Assemble final list
    sortedFiles = outfiles(I);
end

    
    