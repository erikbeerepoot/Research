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
%   /name   CompareImagesByDescriptor
%   /brief  Compares images im1 and im2 by attempting to match SURF
%           features between the frames.

function [score,avgTrackLength] = CompareImagesByDescriptor(im1,im2,doPlot,doSave,mode)
  %Check if we are comparing images of the same size
    if(size(im1)~=size(im2))
        disp('Image sizes dont match');
        return;
    end    

    score = 0;
    avgTrackLength = 0;
    switch(mode)
        case 'SURF'
            [score,avgTrackLength] = CompareBySURFFeatures(im1,im2,doPlot,doSave);
        case 'FAST'
            [score,avgTrackLength] = CompareByFASTFeatures(im1,im2,doPlot,doSave);
        case 'Harris'
            [score,avgTrackLength] = CompareByHarrisFeatures(im1,im2,doPlot,doSave);
        case 'HarrisAffine'
            [score,avgTrackLength] = CompareByHarrisAffineFeatures(im1,im2,doPlot,doSave);
        case 'MSER'
            [score,avgTrackLength] = CompareByMSERRegions(im1,im2,doPlot,doSave);
        otherwise   
            warning('Unexpected feature detector chosen! Options are: SURF,FAST,Harris and HarrisAffine');
    end
end


function [score,avgTrackLength] = CompareBySURFFeatures(im1,im2,doPlot,doSave)
    dirIndex = 1;
    numberOfFeatures = 150;
    
    %Assign default output values
    score = 0;
    avgTrackLength = 0;
        
    %Run AHE
    im1 = adapthisteq(im1);
    im2 = adapthisteq(im2);
    
    surfKp = detectSURFFeatures(im1);
    surfKp2 = detectSURFFeatures(im2);
    
    feat1 = surfKp.selectStrongest(numberOfFeatures);
    feat2 = surfKp2.selectStrongest(numberOfFeatures);
    
      if(length(surfKp)==0 || length(surfKp2)==0)
        disp('No SURF features detected on an image! Skipping...')
        return
    end
    
    if(doPlot==1)
        figure;
        hold on;
        imshow(im1);
        plot(feat1);

        figure;
        hold on;
        imshow(im2);
        plot(feat2);
    end
    
    [score,avgTrackLength] = CompareSURFDescriptors(im1,im2,feat1,feat2,doPlot,doSave);
end

function [score,avgTrackLength] = CompareByFASTFeatures(im1,im2,doPlot,doSave)
    kNumberOfFeatures = 150;
    kFastTreshold = 30;
       
    %Assign default output values
    score = 0;
    avgTrackLength = 0;
    
    %Run AHE
    im1 = adapthisteq(im1);
    im2 = adapthisteq(im2);
   
    %Detect FAST features
    hcornerdet = vision.CornerDetector('Method', 'Local intensity comparison (Rosten & Drummond)'); 
    fastFeatures1 = step(hcornerdet, im1);
    fastFeatures2 = step(hcornerdet, im2);
    
    %Convert to SURFpoints objects
    feat1 = SURFPoints; 
    feat1 = feat1.append(fastFeatures1, 'Scale', 2);
    feat2 = SURFPoints; 
    feat2 = feat2.append(fastFeatures2, 'Scale', 2);
    
    [score,avgTrackLength] = CompareSURFDescriptors(im1,im2,feat1,feat2,doPlot,doSave);
end

function [score,avgTrackLength] = CompareByHarrisFeatures(im1,im2,doPlot,doSave)
    %Assign default output values
    score = 0;
    avgTrackLength = 0;
    
    %Run AHE
    im1 = adapthisteq(im1);
    im2 = adapthisteq(im2);
    
    harrisFeatures1 = detectHarrisFeatures(im1);
    harrisFeatures2 = detectHarrisFeatures(im2);
    
    feat1 = SURFPoints;
    feat1 = feat1.append(harrisFeatures1.Location,'Scale',2);
    feat2 = SURFPoints;
    feat2 = feat2.append(harrisFeatures2.Location,'Scale',2);
    
    [score,avgTrackLength] = CompareSURFDescriptors(im1,im2,feat1,feat2,doPlot,doSave);
end

function [score,avgTrackLength] = CompareByHarrisAffineFeatures(im1,im2,doPlot,doSave)
    detectorRelativePath = '/../contrib/';
    detectorPath = [fileparts(mfilename('fullpath')) detectorRelativePath];

     %Assign default output values
    score = 0;
    avgTrackLength = 0;
    
    %Run AHE
    im1 = adapthisteq(im1);
    im2 = adapthisteq(im2);
    
    %Write intermediate data to temporary storage location
    imwrite(im1, '/tmp/im1.ppm','ppm');
    imwrite(im2, '/tmp/im2.ppm','ppm');
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Harris-Affine %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    [s,w] = unix([[detectorPath '/h_affine.ln'] ' -haraff -i /tmp/im1.ppm -o /tmp/im1.haraff threshold=1000']);
    [harrAffFeat1 nb1 dim1]=loadFeatures('/tmp/im1.haraff');
            
    [s,w] = unix([[detectorPath '/h_affine.ln'] ' -haraff -i /tmp/im2.ppm -o /tmp/im2.haraff threshold=1000']);
    [harrAffFeat2 nb2 dim2]=loadFeatures('/tmp/im2.haraff');

    feat1 = SURFPoints;
    feat1 = feat1.append(harrAffFeat1(1:2,:)','Scale',2);
    feat2 = SURFPoints;
    feat2 = feat2.append(harrAffFeat2(1:2,:)','Scale',2);
    
    [score,avgTrackLength] = CompareSURFDescriptors(im1,im2,feat1,feat2,doPlot,doSave);
end

function [score,avgTrackLength] = CompareByMSERRegions(im1,im2,doPlot,doSave)
    %Assign default output values
    score = 0;
    avgTrackLength = 0;
        
    %Run AHE
    im1 = adapthisteq(im1);
    im2 = adapthisteq(im2);
    
    feat1 = detectMSERFeatures(im1);
    feat2 = detectMSERFeatures(im2);
    
    [score,avgTrackLength] = CompareSURFDescriptors(im1,im2,feat1,feat2,doPlot,doSave);
end

function [score,avgTrackLength] = CompareSURFDescriptors(im1,im2,feat1,feat2,doPlot,doSave)
    kDescThreshold = 0.020;
    kDistanceThreshold = 100;
    
    %Extract features and decide which one to use as query vector
    descs1 = extractFeatures(im1,feat1);
    descs2 = extractFeatures(im2,feat2);

    if(size(descs1,1) < size(descs2,1))
        qDescs = descs1;
        refDescs = descs2;
    else
        qDescs = descs2;
        refDescs = descs1;
    end

    kdtreeNS = KDTreeSearcher(refDescs);

    if(doPlot==1)
        h = figure; clf; imshow(im1); hold on;
    end
    
    featIndex = 1;
    surfMatches.numberOfMatches = 0;
    magnitudeSum = 0;
    while(featIndex < size(qDescs,1))
        %Do nearest neighbour matching 
        neighbourIndex = knnsearch(kdtreeNS,qDescs(featIndex,:));

        % Matching criterion         
        if((1 - qDescs(featIndex,:)*refDescs(neighbourIndex,:)') < kDescThreshold)
            magnitude = 0;
            if(size(descs1,1) < size(descs2,1))
                vector = abs([feat1.Location(featIndex,1),feat1.Location(featIndex,2)] - [feat2.Location(neighbourIndex,1),feat2.Location(neighbourIndex,2)]);
                magnitude = sqrt(vector(1)^2 + vector(2)^2);
                if(magnitude > kDistanceThreshold)
                    featIndex = featIndex + 1;             
                    continue;
                end
                
                
                if(doPlot==1 || doSave==1)
                    line([feat1.Location(featIndex,1),feat2.Location(neighbourIndex,1)],[feat1.Location(featIndex,2),feat2.Location(neighbourIndex,2)],'LineWidth',2,'Color','green');
                end
            else
                vector = abs([feat1.Location(neighbourIndex,1),feat1.Location(neighbourIndex,2)] - [feat2.Location(featIndex,1),feat2.Location(featIndex,2)]);
                magnitude = sqrt(vector(1)^2 + vector(2)^2);
                if(magnitude > kDistanceThreshold) 
                    featIndex = featIndex + 1;             
                    continue;
                end

                if(doPlot==1 || doSave==1)
                    line([feat1.Location(neighbourIndex,1),feat2.Location(featIndex,1)],[feat1.Location(neighbourIndex,2),feat2.Location(featIndex,2)],'LineWidth',2,'Color','green');
                end
            end
            
            magnitudeSum = magnitudeSum + magnitude;
            
            %Match
            surfMatches.numberOfMatches = surfMatches.numberOfMatches + 1;
        end
        featIndex = featIndex + 1;             
    end
    if(doPlot==1)
        hold off;
    end
    
    maxNumFeatures = size(qDescs,1);
    score = surfMatches.numberOfMatches / maxNumFeatures;
    avgTrackLength = magnitudeSum / surfMatches.numberOfMatches;
end

function [feat nb dim]=loadFeatures(file)
    fid = fopen(file, 'r');
    dim=fscanf(fid, '%f',1);
    if dim==1
    dim=0;
    end
    nb=fscanf(fid, '%d',1);
    feat = fscanf(fid, '%f', [5+dim, inf]);
    fclose(fid);
end
