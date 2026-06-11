function json = read_from_json(file_name)

jsonPath = which(file_name);

if isempty(jsonPath)
        error('main:MissingFile', ...
            'Could not find file "%s" on the MATLAB path. Check your config folder.', file_name);
    end

jsonText = fileread(jsonPath);
json = jsondecode(jsonText);

end