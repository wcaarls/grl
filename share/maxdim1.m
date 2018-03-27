function [b, xs] = maxdim1(a, x, d)
%MAXDIM1 Maximum projection over a single dimension

    a = sortrows(a, x);
    xs = unique(a(:,x));
    
    xs = xs(isfinite(xs));
    
    b = zeros(numel(xs), 1);
    
    rr = 1;
    rows = size(a, 1);
    for ii = 1:numel(xs)
        mm = -Inf;
        while rr < rows && a(rr, x) == xs(ii)
            if a(rr, d) > mm
                mm = a(rr, d);
            end
            rr = rr + 1;
        end
        b(ii) = mm;
    end
end
