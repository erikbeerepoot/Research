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


function [score,avgTrackLength] = CompareBySURFFeatures(in_im1,in_im2,doPlot,doSave)
    dirIndex = 1;
    numberOfFeatures = 25;
    
    %Assign default output values
    score = 0;
    avgTrackLength = 0;
        
    %Run AHE
    im1 = adapthisteq(in_im1.intense8Img);
    im2 = adapthisteq(in_im2.intense8Img);
    
    surfKp = detectSURFFeatures(im1);
    surfKp2 = detectSURFFeatures(im2);
    
    RejectMaskedFeatures(surfKp,in_im1.maskImg);
    RejectMaskedFeatures(surfKp2,in_im2.maskImg);
    
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

function [score,avgTrackLength] = CompareByFASTFeatures(in_im1,in_im2,doPlot,doSave)
    kNumberOfFeatures = 150;
    kFastTreshold = 30;
       
    %Assign default output values
    score = 0;
    avgTrackLength = 0;
    
    %Run AHE
    im1 = adapthisteq(in_im1.intense8Img);
    im2 = adapthisteq(in_im2.intense8Img);
    
    %Detect FAST features
    hcornerdet = vision.CornerDetector('Method', 'Local intensity comparison (Rosten & Drummond)'); 
    fastFeatures1 = step(hcornerdet, im1);
    fastFeatures2 = step(hcornerdet, im2);
    
    %Convert to SURFpoints objects
    feat1 = SURFPoints; 
    feat1 = feat1.append(fastFeatures1, 'Scale', 2);
    feat2 = SURFPoints; 
    feat2 = feat2.append(fastFeatures2, 'Scale', 2);
    
    RejectMaskedFeatures(feat1,in_im1.maskImg);
    RejectMaskedFeatures(feat2,in_im2.maskImg);
    
    if(size(feat1,1) > kNumberOfFeatures)
        feat1 = feat1(1:kNumberOfFeatures,:)
    end
    
    if(size(feat2,1) > kNumberOfFeatures)
        feat2 = feat2(1:kNumberOfFeatures,:)
    end
    
    [score,avgTrackLength] = CompareSURFDescriptors(im1,im2,feat1,feat2,doPlot,doSave);
end

function [score,avgTrackLength] = CompareByHarrisFeatures(in_im1,in_im2,doPlot,doSave)
    kNumberOfFeatures = 150;
    
    %Assign default output values
    score = 0;
    avgTrackLength = 0;
    
    %Run AHE
    im1 = adapthisteq(in_im1.intense8Img);
    im2 = adapthisteq(in_im2.intense8Img);
    
    harrisFeatures1 = detectHarrisFeatures(im1);
    harrisFeatures2 = detectHarrisFeatures(im2);
        
    feat1 = SURFPoints;
    feat1 = feat1.append(harrisFeatures1.Location,'Scale',2);
    feat2 = SURFPoints;
    feat2 = feat2.append(harrisFeatures2.Location,'Scale',2);
    
    RejectMaskedFeatures(feat1,in_im1.maskImg);
    RejectMaskedFeatures(feat2,in_im2.maskImg);
    
    if(size(feat1,1) > kNumberOfFeatures)
        feat1 = feat1(1:kNumberOfFeatures,:)
    end
    
    if(size(feat2,1) > kNumberOfFeatures)
        feat2 = feat2(1:kNumberOfFeatures,:)
    end
    
    [score,avgTrackLength] = CompareSURFDescriptors(im1,im2,feat1,feat2,doPlot,doSave);
end

function [score,avgTrackLength] = CompareByHarrisAffineFeatures(in_im1,in_im2,doPlot,doSave)
    detectorRelativePath = '/../contrib/';
    detectorPath = [fileparts(mfilename('fullpath')) detectorRelativePath];

     %Assign default output values
    score = 0;
    avgTrackLength = 0;
    
    %Run AHE
    im1 = adapthisteq(in_im1.intense8Img);
    im2 = adapthisteq(in_im2.intense8Img);
    
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

    
    RejectMaskedFeatures(feat1,in_im1.maskImg);
    RejectMaskedFeatures(feat2,in_im2.maskImg);
    
    if(size(feat1,1) > kNumberOfFeatures)
        feat1 = feat1(1:kNumberOfFeatures,:)
    end
    
    if(size(feat2,1) > kNumberOfFeatures)
        feat2 = feat2(1:kNumberOfFeatures,:)
    end
    
    [score,avgTrackLength] = CompareSURFDescriptors(im1,im2,feat1,feat2,doPlot,doSave);
end

function [score,avgTrackLength] = CompareByMSERRegions(in_im1,in_im2,doPlot,doSave)
    kNumberOfFeatures = 150; 
    
    %Assign default output values
    score = 0;
    avgTrackLength = 0;
        
    %Run AHE
    im1 = adapthisteq(in_im1.intense8Img);
    im2 = adapthisteq(in_im2.intense8Img);
    
    
    MSERFeat1 = detectMSERFeatures(im1);
    MSERFeat2 = detectMSERFeatures(im2);
    
    feat1 = SURFPoints;
    feat1 = feat1.append(MSERFeat1.Location,'Scale',2);
    feat2 = SURFPoints;
    feat2 = feat2.append(MSERFeat2.Location,'Scale',2);
    
    RejectMaskedFeatures(feat1,in_im1.maskImg);
    RejectMaskedFeatures(feat2,in_im2.maskImg);
    
    if(size(feat1,1) > kNumberOfFeatures)
        feat1 = feat1(1:kNumberOfFeatures,:)
    end
    
    if(size(feat2,1) > kNumberOfFeatures)
        feat2 = feat2(1:kNumberOfFeatures,:)
    end
    
    
    [score,avgTrackLength] = CompareSURFDescriptors(im1,im2,feat1,feat2,doPlot,doSave);
end

function [score,avgTrackLength] = CompareSURFDescriptors(im1,im2,feat1,feat2,doPlot,doSave)
    kDescThreshold = 0.020;
    kDistanceThreshold = 100;
    
    %Extract features and decide which one to use as query vector
    descs1 = extractFeatures(im1,feat1);
    descs2 = extractFeatures(im2,feat2);
    qDescs = descs2;
    refDescs = descs1;
 
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
            vector = abs([feat1.Location(neighbourIndex,1),feat1.Location(neighbourIndex,2)] - [feat2.Location(featIndex,1),feat2.Location(featIndex,2)]);
            magnitude = sqrt(vector(1)^2 + vector(2)^2);
            
            %Reject egregious outliers
            if(magnitude > kDistanceThreshold) 
                featIndex = featIndex + 1;             
                continue;
            end

            if(doPlot==1 || doSave==1)
                line([feat1.Location(neighbourIndex,1),feat2.Location(featIndex,1)],[feat1.Location(neighbourIndex,2),feat2.Location(featIndex,2)],'LineWidth',2,'Color','green');
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

function outFeats = RejectMaskedFeatures(feats,mask)
    idx = 1;
    outFeats = SURFPoints;
    for k = 1 : size(feats,1)
        x = round(feats.Location(k,1));
        y = round(feats.Location(k,2));
        if((x < size(mask,1)) && (y < size(mask,2)) && (mask(x,y)==1))
           outFeats(idx) = feats(k);
           idx = idx + 1;
        end
    end
end