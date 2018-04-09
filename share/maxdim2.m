function [b, xs, ys] = maxdim2(a, x, y, d)
%MAXDIM2 Maximum projection over two dimensions

    a = sortrows(a, [x y]);
    xs = unique(a(:,x));
    ys = unique(a(:,y));
    
    xs = xs(isfinite(xs));
    ys = ys(isfinite(ys));
    
    b = zeros(numel(xs), numel(ys));
    
    rr = 1;
    rows = size(a, 1);
    for ii = 1:numel(xs)
        for jj = 1:numel(ys)
            mm = -Inf;
            while rr < rows && all(a(rr, [x y]) == [xs(ii) ys(jj)])
                if a(rr, d) > mm
                    mm = a(rr, d);
                end
                rr = rr + 1;
            end
            b(ii,jj) = mm;
        end
    end

end
