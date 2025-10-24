function blocks = segmentBlocks(rgb)
% returns array of structs: color, centroid [u v], bbox, area
hsv = rgb2hsv(rgb);
H=hsv(:,:,1); S=hsv(:,:,2); V=hsv(:,:,3);

ranges.red   = [0.00 0.03; 0.60 1.00]; % wrap red
ranges.green = [0.25 0.45];
ranges.blue  = [0.55 0.75];

blocks = [];
colors = fieldnames(ranges);
for c = 1:numel(colors)
    name = colors{c};
    if strcmp(name,'red')
        mask = ((H>=ranges.red(1,1) & H<=ranges.red(1,2)) | ...
                (H>=ranges.red(2,1) & H<=ranges.red(2,2))) & S>0.35 & V>0.25;
    else
        r = ranges.(name);
        mask = H>=r(1) & H<=r(2) & S>0.35 & V>0.25;
    end
    mask = bwareaopen(imclose(mask, strel('disk',3)), 150);
    stats = regionprops(mask,'Centroid','BoundingBox','Area');
    for k=1:numel(stats)
        blocks(end+1) = struct('color',name, ...
                               'centroid',stats(k).Centroid, ...
                               'bbox',stats(k).BoundingBox, ...
                               'area',stats(k).Area); %#ok<AGROW>
    end
end
end
