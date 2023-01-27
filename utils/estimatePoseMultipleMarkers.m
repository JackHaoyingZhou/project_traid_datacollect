% *************************************************************************
% estimate probe pose using marker poses
% *************************************************************************
function [probe_pos, mk_trans] = estimatePoseMultipleMarkers(mk_rot, mk_trans)

transducer_length = 0.033;  % [m]
transducer_width = 0.008;   % [m]
transducer_depth = 0.007;   % [m]   % TODO: add depth offset directly in PA image

num_mk_detected = sum(sum(isnan(mk_trans),2) == 0);
fprintf('detected markers: %d\n', num_mk_detected);
if num_mk_detected < 3
    mk_trans = completeMarkerPos(mk_rot, mk_trans);
end

probe_pos = eye(4);

x_vecs = [mk_trans(1,:) - mk_trans(2,:); ...
    mk_trans(3,:) - mk_trans(4,:)];
x_vec = normalize(mean(x_vecs, 'omitnan'), 'norm');

y_vecs = [mk_trans(1,:) - mk_trans(3,:); ...
    mk_trans(2,:) - mk_trans(4,:)];
y_vec = normalize(mean(y_vecs, 'omitnan'), 'norm');

% z_vec = estimateNormal(mk_trans);
z_vec = cross(x_vec, y_vec);

%     orig = mean(mk_trans,'omitnan');  % this only works when all markers are visible

orig = zeros(3, 1);
x1 = mk_trans(1,1) - x_vec*transducer_length/2;
x2 = mk_trans(2,1) + x_vec*transducer_length/2;
x3 = mk_trans(3,1) - x_vec*transducer_length/2;
x4 = mk_trans(4,1) + x_vec*transducer_length/2;
x12 = mean([x1, x2]);
x34 = mean([x3, x4]);
orig(1) = mean([x12, x34], 'omitnan');

y1 = mk_trans(1,2) + y_vec*transducer_width/2;
y2 = mk_trans(2,2) - y_vec*transducer_width/2;
y3 = mk_trans(3,2) + y_vec*transducer_width/2;
y4 = mk_trans(4,2) - y_vec*transducer_width/2;
y13 = mean([y1, y3]);
y24 = mean([y2, y4]);
orig(2) = mean([y13, y24], 'omitnan');

z1 = mk_trans(1,3) + z_vec*transducer_depth;
z2 = mk_trans(2,3) + z_vec*transducer_depth;
z3 = mk_trans(3,3) + z_vec*transducer_depth;
z4 = mk_trans(4,3) + z_vec*transducer_depth;
z14 = mean([z1, z4]);
z23 = mean([z2, z3]);
orig(3) = mean([z14, z23],'omitnan');

probe_pos(1:3, 1) = x_vec;
probe_pos(1:3, 2) = y_vec;
probe_pos(1:3, 3) = z_vec;
probe_pos(1:3, 4) = orig;

end