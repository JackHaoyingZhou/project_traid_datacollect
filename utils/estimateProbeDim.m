% *************************************************************************
% estimate probe length & probe width
% *************************************************************************
function [probe_length, probe_width] = estimateProbeDim(mk_trans)

    length1 = norm(mk_trans(1,:) - mk_trans(2,:));
    width1 = norm(mk_trans(1,:) - mk_trans(3,:));
    length2 = norm(mk_trans(3,:) - mk_trans(4,:));
    width2 = norm(mk_trans(2,:) - mk_trans(4,:));
    fprintf('probe length: %f, %f, probe width: %f, %f\n', ...
        length1, length2, width1, width2)
    
    probe_length = mean([length1, length2], 'omitnan');
    probe_width = mean([width1, width2], 'omitnan');
    
end

