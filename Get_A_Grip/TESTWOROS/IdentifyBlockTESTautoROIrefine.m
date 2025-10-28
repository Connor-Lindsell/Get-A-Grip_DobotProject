function IdentifyBlockTESTautoROIrefine
    disp('=======================================');
    disp('Identifying block in the scene...');

    I    = imread('frame0001.jpg');
    gray = rgb2gray(I);

    % === STEP 1: auto ROI (yours, unchanged) ===
    edges0 = edge(gray, 'Canny', [0.05 0.2]);
    bw = imdilate(edges0, strel('disk', 2));
    bw = imclearborder(bw);

    stats = regionprops(bw, 'BoundingBox','Area','MajorAxisLength','MinorAxisLength','Solidity','Extent');
    roi = [1 1 size(gray,2) size(gray,1)]; % fallback
    if ~isempty(stats)
        imgArea = numel(gray);
        valid = [stats.Area] < 0.25*imgArea & [stats.Area] > 300;
        stats = stats(valid);
        if ~isempty(stats)
            scores = arrayfun(@(s) ...
                (1 ./ (1 + abs((s.MajorAxisLength / max(s.MinorAxisLength,1)) - 1))) .* ...
                (s.Area / (s.BoundingBox(3)*s.BoundingBox(4))), stats);
            [~,idx] = max(scores);
            roi = round(stats(idx).BoundingBox);
        end
    end

    % === STEP 2: Harris (for viz only) ===
    pts = detectHarrisFeatures(gray,'ROI',roi,'MinQuality',0.05);
    xyH  = double(pts.Location);

    % === STEP 3: edge processing inside ROI and OUTER-BAND selection ===
    pad = 12;                                 % small padding around ROI
    x1 = max(1, roi(1)-pad);
    y1 = max(1, roi(2)-pad);
    x2 = min(size(gray,2), roi(1)+roi(3)+pad);
    y2 = min(size(gray,1), roi(2)+roi(4)+pad);

    patch = gray(y1:y2, x1:x2);
    patch = adapthisteq(patch);               % stabilise local contrast
    E = edge(patch, 'Canny');                 % edges in the patch

    % Close gaps strongly to make the outer frame continuous
    E = imdilate(E, strel('line',5,0));
    E = imdilate(E, strel('line',5,90));
    E = imclose(E, strel('disk',3));
    E = bwmorph(E, 'bridge');
    E = bwmorph(E, 'clean');

    % Keep only an outer band of the ROI to reject interior features
    [yy, xx] = ndgrid(1:size(patch,1), 1:size(patch,2));
    cx = (size(patch,2)+1)/2; cy = (size(patch,1)+1)/2;
    r = hypot(xx-cx, yy-cy);
    r = r / max(r(:));
    outerMask = r > 0.55;                     % keep only outer 45% band
    E = E & outerMask;

    % Collect edge coordinates in image space
    [ey, ex] = find(E);
    if numel(ex) < 20
        figure; imshow(gray); hold on;
        rectangle('Position',roi,'EdgeColor','y','LineWidth',2);
        if ~isempty(xyH), scatter(xyH(:,1), xyH(:,2), 12, 'g', 'filled'); end
        title('Too few outer-band edges to form quad');
        return
    end
    ptsOuter = [ex + x1 - 1, ey + y1 - 1];    % back to full-image coords

    % === STEP 4: convex hull then true minimum-area rectangle ===
    k = convhull(double(ptsOuter(:,1)), double(ptsOuter(:,2)));
    hullPts = ptsOuter(k,:);
    rect = minAreaRect_xy(hullPts);           % 4x2 corners

    % === STEP 5: order corners TL, TR, BR, BL ===
    corners = orderCorners(rect);

    % Optional small outward expansion to hug the very outer edge
    expand_px = 2;
    corners = expandQuadOutward(corners, expand_px);

    % === STEP 6: visualise ===
    figure; imshow(gray); hold on;
    rectangle('Position',roi,'EdgeColor','y','LineWidth',2);
    if ~isempty(xyH), scatter(xyH(:,1), xyH(:,2), 12, 'g', 'filled'); end
    plot([corners(:,1); corners(1,1)], [corners(:,2); corners(1,2)], 'r-', 'LineWidth', 2);
    scatter(corners(:,1), corners(:,2), 40, 'r', 'filled');
    title('Refined tag outline from outer-band edges');

    assignin('base','tagCorners', corners); % TL, TR, BR, BL
end

function rect = minAreaRect_xy(points)
    % Rotating calipers min-area rectangle for 2D points (Nx2)
    P = double(points);
    k = convhull(P(:,1), P(:,2));
    H = P(k,:);
    E = diff([H; H(1,:)]);                 % edges
    ang = atan2(E(:,2), E(:,1));
    ang = mod(ang, pi/2);
    ang = unique(ang);

    bestArea = inf; bestRect = [];
    for a = ang'
        R = [cos(a) sin(a); -sin(a) cos(a)];
        Rpts = H * R';
        minX = min(Rpts(:,1)); maxX = max(Rpts(:,1));
        minY = min(Rpts(:,2)); maxY = max(Rpts(:,2));
        A = (maxX - minX) * (maxY - minY);
        if A < bestArea
            bestArea = A;
            rectLocal = [minX minY;
                         maxX minY;
                         maxX maxY;
                         minX maxY];
            bestRect = rectLocal * R;     % back to image frame
        end
    end
    rect = bestRect;
end

function corners = orderCorners(c)
    % order TL, TR, BR, BL
    C = mean(c,1);
    ang = atan2(c(:,2)-C(2), c(:,1)-C(1));
    [~,ord] = sort(ang);              % CCW
    c = c(ord,:);
    [~, tl] = min(c(:,2) + 0.001*c(:,1));
    corners = circshift(c, -(tl-1), 1);
end

function Q = expandQuadOutward(corners, d)
    % Expand quad outward by d pixels along outward normals
    Q = zeros(size(corners));
    C = mean(corners,1);
    for i = 1:4
        j = mod(i,4)+1;
        e  = corners(j,:) - corners(i,:);
        n  = [-e(2), e(1)];                   % edge normal
        n  = n / max(norm(n),1e-9);
        sgn = sign(dot(corners(i,:) - C, n)); % outward direction
        Q(i,:) = corners(i,:) + sgn*d*n;
    end
end


