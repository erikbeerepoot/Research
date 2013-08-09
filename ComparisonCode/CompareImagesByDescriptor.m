function [score,avgTrackLength] = CompareImagesByDescriptor(im1,im2,doPlot,doSave,fileNamePostFix)
    kDistanceThreshold = 100;
    kDescThreshold = 0.025;
    dirIndex = 1;
    numberOfFeatures = 100;
    
    %Assign default output values
    score = 0;
    avgTrackLength = 0;
    
    %Check if we are comparing images of the same size
    if(size(im1)~=size(im2))
        disp('Image sizes dont match');
        return;
    end
    
    if(doPlot==1)
        %Display images    
        figure; clf;
        imshow(im1);

        figure; clf;
        imshow(im2);
    end
    
    %Compute pixel differences
    mask = zeros([size(im1),3]);
    for row = 1 : size(im1,1)
        mask(row,:,1) = (im1(row,:)~=im2(row,:));
    end
    
    %Run AHE
    im1 = adapthisteq(im1);
    im2 = adapthisteq(im2);
    
    %make sure LD_LIBRARY_PATH has cuda libs
    setenv LD_LIBRARY_PATH /opt/MATLAB/R2011a/sys/os/glnxa64:/opt/MATLAB/R2011a/bin/glnxa64:/opt/MATLAB/R2011a/extern/lib/glnxa64:/opt/MATLAB/R2011a/runtime/glnxa64:/opt/MATLAB/R2011a/sys/java/jre/glnxa64/jre/lib/amd64/native_threads:/opt/MATLAB/R2011a/sys/java/jre/glnxa64/jre/lib/amd64/server:/opt/MATLAB/R2011a/sys/java/jre/glnxa64/jre/lib/amd64:/usr/local/cuda-4.2/lib64/
    
    % Generate .ppm
    imwrite(im1, ['/home/eeb/' 'im1.ppm'],'ppm');                
    [s,w] = unix([ ['/home/eeb/ros_workspace/asrl_algorithms/gpusurf/bin/' '/gpusurf_engine'] ' --image ' ['/home/eeb/' 'im1.ppm']]);
    [surfKp] = load_gpu_keypoints(['/home/eeb/im1-gpusurf.key']);
    
    
    imwrite(im2, ['/home/eeb/' 'im2.ppm'],'ppm');                
    [s,w] = unix([ ['/home/eeb/ros_workspace/asrl_algorithms/gpusurf/bin/' '/gpusurf_engine'] ' --image ' ['/home/eeb/' 'im2.ppm']]);
    [surfKp2] = load_gpu_keypoints(['/home/eeb/im2-gpusurf.key']);
    
    if(doPlot==1)
        figure;
        hold on;
        surf_plot_keypoints(im1,surfKp,numberOfFeatures);

        figure;
        hold on;
        surf_plot_keypoints(im2,surfKp2,numberOfFeatures);
    end
    
    if(length(surfKp)==0 || length(surfKp2)==0)
        disp('No SURF features detected!')
        return
    end
    %Select N best matches
    feat1 = surf_best_n_keypoints(surfKp,numberOfFeatures);
    feat2 = surf_best_n_keypoints(surfKp2,numberOfFeatures);
   
    %Build integral images (for SURF desc)
    iimg1 = surf_build_iimg(im1);
    iimg2 = surf_build_iimg(im2);

    featIndex = 1;
   
    while(featIndex <= size(feat1,1));              
        %Compute descriptors
        Options = surf_init_options();
        %kp.orientation = surf_find_orientation(iimg1, feat1(featIndex), Options);
        descs1(featIndex,:) = surf_descriptor(iimg1, feat1(featIndex), Options);
        featIndex = featIndex + 1;
    end

    featIndex = 1;
    while(featIndex <= size(feat2,1))     

        %Compute descriptors                    
        Options = surf_init_options();
        %kp.orientation = surf_find_orientation(iimg2, kp, Options);
        descs2(featIndex,:) = surf_descriptor(iimg2, feat2(featIndex), Options);
        featIndex = featIndex + 1;
    end

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
                vector = abs([feat1(featIndex).x,feat1(featIndex).y] - [feat2(neighbourIndex).x,feat2(neighbourIndex).y]);
                magnitude = sqrt(vector(1)^2 + vector(2)^2);
                if(magnitude > kDistanceThreshold)
                    featIndex = featIndex + 1;             
                    continue;
                end
                
                
                if(doPlot==1 || doSave==1)
                    line([feat1(featIndex).x,feat2(neighbourIndex).x],[feat1(featIndex).y,feat2(neighbourIndex).y],'LineWidth',2,'Color','green');
                end
                else
                 vector = abs([feat1(neighbourIndex).x,feat1(neighbourIndex).y] - [feat2(featIndex).x,feat2(featIndex).y]);
                magnitude = sqrt(vector(1)^2 + vector(2)^2);
                if(magnitude > kDistanceThreshold) 
                    featIndex = featIndex + 1;             
                    continue;
                end
                    
                if(doPlot==1 || doSave==1)
                    line([feat1(neighbourIndex).x,feat2(featIndex).x],[feat1(neighbourIndex).y,feat2(featIndex).y],'LineWidth',2,'Color','green');
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

    if(doSave==1)
        print(h,['~/Dropbox/Research/Images/test comp framework/simmatches/matches-' fileNamePostFix],'-depsc');
    end
    
    avgTrackLength = magnitudeSum / surfMatches.numberOfMatches;
end