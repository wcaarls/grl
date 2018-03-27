function manyplots(a, desc, mag)
%MANYPLOTS Grid of maximum projections
%
%   MANYPLOTS(A, DESC) plots maximum projections of parameter
%   realizations against eachother. DESC is a list of N parameter
%   names, A is an (N+1)xM matrix, the first N colums being the parameter
%   values of the experiment, and the last one the dependent variable.
%
%   MANUPLOTS(A, DESC, MAG) sets the plot size magnification to
%   avoid empty space. Default 1.2.

    if nargin < 3
        mag = 1.2;
    end

    dims = numel(desc);
    
    % Find extents for one-dimensional and two-dimensional plots
    minzz1 = Inf;
    maxzz1 = -Inf;
    minzz2 = Inf;
    maxzz2 = -Inf;
    
    for ii = 1:dims
        for jj = 1:dims
            if ii == jj
                zz = maxdim1(a, ii, dims+1);
                zz = zz(isfinite(zz));
                
                minzz1 = min(minzz1, min(zz));
                maxzz1 = max(maxzz1, max(zz));
            else
                zz = maxdim2(a, ii, jj, dims+1);
                zz = zz(isfinite(zz));
                
                minzz2 = min(minzz2, min(zz));
                maxzz2 = max(maxzz2, max(zz));
            end
        end
    end

    % Plot all combinations
    for ii = 1:dims
        for jj = 1:dims
            fprintf('%s vs %s\n', desc{jj}, desc{ii});
            
            subplot(dims, dims, (ii-1)*dims+jj)
            if ii == jj
                % Diagonal: 1d plot
                [zz, xs] = maxdim1(a, ii, dims+1);
                
                plot(xs, zz);
                axis tight;
                ax = axis;
                axis([ax(1), ax(2), minzz1, maxzz1]);
            else
                % Off-diagonal: 2d plot
                [zz, xs, ys] = maxdim2(a, ii, jj, dims+1);
                
                [xx,yy] = meshgrid(ys, xs);
                surf(xx, yy, zz);
                axis tight;
                ax = axis;
                axis([ax(1), ax(2), ax(3), ax(4), minzz2, maxzz2, minzz2, maxzz2]);
                shading interp
                view(0,90);
            end
            
            % Only draw text along outer edge
            if ii == dims
                xlabel(desc{jj}, 'Interpreter', 'none')
            else
                set(gca, 'xtick', []);
            end
            if jj == 1
                ylabel(desc{ii}, 'Interpreter', 'none')
            else
                set(gca, 'ytick', []);
            end
            
            % Remove some empty space. Adjust as necessary.
            pos = get(gca, 'InnerPosition');
            set(gca, 'InnerPosition', [pos(1), pos(2), pos(3)*mag, pos(4)*mag]);
        end
    end

end
