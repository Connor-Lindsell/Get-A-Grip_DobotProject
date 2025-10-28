function IdentifyBlockTESTmanualinput
    disp('=======================================');
    disp('Identifying block in the scene...');

    I = imread('frame0001.jpg'); % For testing without ROS

    gray = rgb2gray(I);

    % 1) Pick an ROI once (interactive) or hardcode it later
    figure;
    imshow(gray); title('Draw a box around the tag, double click to finish');
    h = drawrectangle;    % interactive
    roi = round(h.Position);  % [x y w h]

    % 2) Detect Harris corners only inside ROI
    pts = detectHarrisFeatures(gray,'ROI',roi,'MinQuality',0.1,'FilterSize',5);

    % 3) Visualize
    hold on;
    rectangle('Position',roi,'EdgeColor','y','LineWidth',2);
    if ~isempty(pts)
        strongest = pts.selectStrongest(min(50,pts.Count));
        plot(strongest);
        title('Harris inside ROI');
    else
        title(ax,'No features found inside ROI');
    end

    
end