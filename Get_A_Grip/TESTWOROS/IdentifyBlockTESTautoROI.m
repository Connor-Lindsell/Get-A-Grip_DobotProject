function IdentifyBlockTESTautoROI
    disp('=======================================');
    disp('Identifying block in the scene...');

    I    = imread('frame0001.jpg'); 
    gray = rgb2gray(I);

    % === STEP 1: automatically find a square-ish bright region (the tag) ===
    % replace your bw creation lines with:
    edges = edge(gray, 'Canny', [0.05 0.2]);   % detects high-contrast edges
    bw = imdilate(edges, strel('disk', 2));     % slightly thicken edges
    bw = imclearborder(bw);



    stats = regionprops(bw, 'BoundingBox','Area','MajorAxisLength','MinorAxisLength','Solidity','Extent');
    roi = [1 1 size(gray,2) size(gray,1)]; % fallback if nothing found

    if ~isempty(stats)
        imgArea = numel(gray);
        valid = [stats.Area] < 0.25*imgArea & [stats.Area] > 300; % only mid-sized blobs
        stats = stats(valid);

        if ~isempty(stats)
            % Score by square-ness and compactness, NOT size
            scores = arrayfun(@(s) ...
            (1 ./ (1 + abs((s.MajorAxisLength / max(s.MinorAxisLength,1)) - 1))) .* ...
            (s.Area / (s.BoundingBox(3)*s.BoundingBox(4))), stats);

            
            [~,idx] = max(scores);
            roi = round(stats(idx).BoundingBox);

        end
    end

    % === STEP 2: detect Harris features inside that ROI ===
    pts = detectHarrisFeatures(gray,'ROI',roi,'MinQuality',0.05);

    % === STEP 3: visualize ===
    figure; imshow(gray); hold on;
    rectangle('Position',roi,'EdgeColor','y','LineWidth',2);
    if ~isempty(pts)
        plot(pts);
        title('Harris inside automatically detected ROI');
    else
        title('No Harris features found in ROI');
    end
end