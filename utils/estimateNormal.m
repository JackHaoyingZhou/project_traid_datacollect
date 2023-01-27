% *************************************************************************
% estimate normal vector
% *************************************************************************
function norm = estimateNormal(points, method)

if nargin < 2
    method = 2;
end

if method == 1
    vecs = points - orig;
    norms = zeros(size(vecs,1),3);
    for i = 1:size(vecs, 1)
        if i == length(points)
            norms(i,:) = -normalize(cross(vecs(i,:), vecs(1,:)), 'norm');
        else
            norms(i,:) = -normalize(cross(vecs(i,:), vecs(i+1,:)), 'norm');
        end
        if norms(i, 3) < 0
            norms(i, :) = - norms(i, :);
        end
    end

elseif method == 2
    norms = zeros(size(points,1), 3);
    P1_ind = [2, 3, 4, 1];
    P2_ind = [3, 4, 1, 2];
    for i = 1:size(norms,1)
        P0 = points(i,:);
        P1 = points(P1_ind(i), :);
        P2 = points(P2_ind(i), :);
        norms(i, :) = normalize(cross(P1-P0, P2-P0), 'norm');
        if norms(i, 3) < 0
            norms(i, :) = - norms(i, :);
        end
    end
end

norm = mean(norms, 'omitnan');

end

