f = dir('frame*.jpg');
fil = {f.name};

if isempty(fil)
    disp('No .jpg files found.');
end

for k = 1:numel(fil)
    file = fil{k};
    new_file = strrep(file, '.jpg', '.png');
    
    disp(['Converting ', file, ' → ', new_file]);
    
    try
        im = imread(file);
        imwrite(im, new_file);
    catch ME
        warning(['Failed to convert ', file, ': ', ME.message]);
    end
end
