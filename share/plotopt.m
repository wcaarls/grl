function plotopt(file)
%PLOTOPT Plot grlo optimization result
%
%   PLOTOPT(FILE) visualizes all experiments run during optimization.
%   The FILE should have a header with parameter names and be of the form
%   "par1val", "par2val", ..., "parNval", runs, mean, stderr, stddev

    a = importdata(file);
    
    % Extract descriptions from header. Somehow, the entire header is read
    % into the first element of the first line...
    descall = a.textdata{1,1};
    descall = descall(descall~='"' & descall~=' ');
    desc = strsplit(descall, ',');
    desc = desc(1:size(a.textdata, 2));
    
    % Find common part of parameter names
    common = desc{1};
    for ii = 2:numel(desc)
        d = desc{ii};
        
        if numel(d) < numel(common)
            common = common(1:numel(d));
        else
            d = d(1:numel(common));
        end
        
        diff = find(d ~= common, 1);
        if ~isempty(diff)
            common = common(1:diff-1);
        end
    end
    
    % Remove common part of parameter names
    for ii = 1:numel(desc)
        desc{ii} = desc{ii}(numel(common)+1:end);
    end
    
    % Create data array by taking first number-like part from cells
    data = ones(size(a.data, 1), size(a.textdata, 2));
    
    for rr = 1:size(a.data, 1)
        for cc = 1:size(a.textdata, 2)
            elem = a.textdata{rr+1, cc};
            
            upto = find(~isstrprop(elem, 'digit') & ...
                        ~isstrprop(elem, 'wspace') & ...
                        elem ~= '.', 1);
            if ~isempty(upto)
                elem = elem(1:upto-1);
            end
            
            if ~isempty(elem)
                data(rr,cc) = str2double(elem);
            end
        end
    end
    
    % Append performance data
    data = [data a.data(:,2)];
    
    % Do plots
    manyplots(data, desc(1:size(a.textdata, 2)));
    
end
