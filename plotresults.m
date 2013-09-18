
    %Processed rotational speed
    rotSpeed = [v(1),mean([v(2),v(3)]),mean([v(4),v(5)]),v(6),v(7),mean([v(8),v(9)]),v(10)]
    
    %%%%%%%%%%%%%%%%% SURF %%%%%%%%%%%%%%%%%%%
    %Compute mean matching scores
    meanCompScore = (sum(surfCompScore') ./ sum(surfCompScore'>0));
    meanDistortScore = (sum(surfDistortScore') ./ sum(surfDistortScore'>0));
    meanTrackLength = (sum(compTrackLength') ./ sum(compTrackLength'>0));
    meanDistortTrackLength = (sum(distortTrackLength') ./ sum(distortTrackLength'>0));
    
    for idx = 1 : size(surfCompScore,1)
        stdCompScore(idx) = std(surfCompScore(idx,surfCompScore(idx,:) > 0))
        stdDistortScore(idx) = std(surfDistortScore(idx,surfDistortScore(idx,:) > 0))
        stdTrackLength(idx) = std(compTrackLength(idx,compTrackLength(idx,:) > 0))
        stdDistortTrackLength(idx) = std(distortTrackLength(idx,distortTrackLength(idx,:) > 0))
    end
    
    processedCompScore = [meanCompScore(1), mean([meanCompScore(2) meanCompScore(3)]), mean([meanCompScore(4) meanCompScore(5)]), meanCompScore(6),meanCompScore(7), mean([meanCompScore(8),meanCompScore(9)]),meanCompScore(10)]
    processedCompScoreStd = [stdCompScore(1), mean([stdCompScore(2) stdCompScore(3)]), mean([stdCompScore(4) stdCompScore(5)]), stdCompScore(6),stdCompScore(7), mean([stdCompScore(8),stdCompScore(9)]),stdCompScore(10)]
    processedDistortScore = [meanDistortScore(1), mean([meanDistortScore(2),meanDistortScore(3)]), mean([meanDistortScore(4),meanDistortScore(5)]), meanDistortScore(6),meanDistortScore(7), mean([meanDistortScore(8),meanDistortScore(9)]),meanDistortScore(10)]
    processedDistortScoreStd = [stdDistortScore(1), mean([stdDistortScore(2),stdDistortScore(3)]), mean([stdDistortScore(4),stdDistortScore(5)]), stdDistortScore(6),stdDistortScore(7), mean([stdDistortScore(8),stdDistortScore(9)]),stdDistortScore(10)]
    
    processedCompLength = [meanTrackLength(1), mean([meanTrackLength(2), meanTrackLength(3)]), mean([meanTrackLength(4), meanTrackLength(5)]), meanTrackLength(6),meanTrackLength(7), mean([meanTrackLength(8),meanTrackLength(9)]),meanTrackLength(10)]
    processedDistortLength = [meanDistortTrackLength(1), mean([meanDistortTrackLength(2),meanDistortTrackLength(3)]), mean([meanDistortTrackLength(4),meanDistortTrackLength(5)]), meanDistortTrackLength(6),meanDistortTrackLength(7), mean([meanDistortTrackLength(8),meanDistortTrackLength(9)]),meanDistortTrackLength(10)]
    processedCompLengthStd = [stdTrackLength(1), mean([stdTrackLength(2), stdTrackLength(3)]), mean([stdTrackLength(4), stdTrackLength(5)]), stdTrackLength(6),stdTrackLength(7), mean([stdTrackLength(8),stdTrackLength(9)]),stdTrackLength(10)]
    processedDistortLengthStd = [stdDistortTrackLength(1), mean([stdDistortTrackLength(2),stdDistortTrackLength(3)]), mean([stdDistortTrackLength(4),meanDistortTrackLength(5)]), stdDistortTrackLength(6),stdDistortTrackLength(7), mean([stdDistortTrackLength(8),stdDistortTrackLength(9)]),stdDistortTrackLength(10)]
    
    %Plot matching score
    figure(1); clf; hold on;
    plot(abs(rotSpeed),processedCompScore,'b-x');
    plot(abs(rotSpeed),processedDistortScore,'r-x')
    legend('Compensated score','Distorted score');
    xlabel('Rotational speed (rad/s)');
    ylabel('Normalized matching score');
    
    %Plot errorbars
    errorbar(abs(rotSpeed),processedCompScore,processedCompScoreStd,'b');
    errorbar(abs(rotSpeed),processedDistortScore,processedDistortScoreStd,'r');
    
    
    %Plot track length
    figure(2); clf; hold on;
    plot(abs(rotSpeed),processedCompLength,'b-x');
    plot(abs(rotSpeed),processedDistortLength,'r-x')
    legend('Compensated score','Distorted score');
    xlabel('Rotational speed (rad/s)');
    ylabel('Track length (pixels)');
    
    %Plot errorbars
    errorbar(abs(rotSpeed),processedCompLength,processedCompLengthStd,'b');
    errorbar(abs(rotSpeed),processedDistortLength,processedDistortLengthStd,'r');
    
    figure(3); clf; hold on;
    plot(surfCompScore(1,:),'b');
    plot(surfDistortScore(1,:),'r');
    hold off;
    
    figure(4); clf; hold on;
    plot(surfCompScore(2,surfCompScore(2,:)>0),'b');
    plot(surfDistortScore(2,surfDistortScore(2,:)>0),'r');
    hold off;
    
    figure(5); clf; hold on;
    plot(surfCompScore(4,surfCompScore(4,:)>0),'b');
    plot(surfDistortScore(4,surfDistortScore(4,:)>0),'r');
    hold off;
    
    figure(6); clf; hold on;
    plot(surfCompScore(6,surfCompScore(6,:)>0),'b');
    plot(surfDistortScore(6,surfDistortScore(6,:)>0),'r');
    hold off;