% *************************************************************************
% complete marker positions (x,y,z) based on known marker spatial relations
% *************************************************************************
function [mk_trans_complete] = completeMarkerPos(mk_rot, mk_trans)

rotx = @(t) [1 0 0; 0 cosd(t) -sind(t) ; 0 sind(t) cosd(t)];
% roty = @(t) [cosd(t) 0 sind(t) ; 0 1 0 ; -sind(t) 0  cosd(t)];
% rotz = @(t) [cosd(t) -sind(t) 0 ; sind(t) cosd(t) 0 ; 0 0 1];

transducer_length = 0.033;  % [m]
transducer_width = 0.008;   % [m]
theta = -30; % -120;

mk_ids = 1:size(mk_trans, 1);
mk_detected = mk_ids(sum(isnan(mk_trans),2) == 0);

mk_trans_complete = mk_trans;

for i = 1:length(mk_detected)
    mk_id = mk_detected(i);
    switch mk_id
        case 1
            % calc mk2 position
            mk2 = mk_trans_complete(mk_id,:) - mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(2,:) = mean([mk2; mk_trans_complete(2,:)], 'omitnan');
            % calc mk3 position
            R = mk_rot(:,:,mk_id) * rotx(theta);
            mk3 = mk_trans_complete(mk_id,:) + R(:,2)' * transducer_width;
            mk_trans_complete(3,:) = mean([mk3; mk_trans_complete(3,:)], 'omitnan');
            % calc mk4 position
            mk4 = mk3 - mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(4,:) = mean([mk4; mk_trans_complete(4,:)], 'omitnan');

        case 2
            % calc mk1 position
            mk1 = mk_trans_complete(mk_id,:) + mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(1,:) = mean([mk1; mk_trans_complete(1,:)], 'omitnan');
            % calc mk4 position
            R = mk_rot(:,:,mk_id) * rotx(theta);
            mk4 = mk_trans_complete(mk_id,:) + R(:,2)' * transducer_width;
            mk_trans_complete(4,:) = mean([mk4; mk_trans_complete(4,:)], 'omitnan');
            % calc mk3 position
            mk3 = mk4 + mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(3,:) = mean([mk3; mk_trans_complete(3,:)], 'omitnan');

        case 3
            % calc mk4 position
            mk4 = mk_trans_complete(mk_id,:) - mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(4,:) = mean([mk4; mk_trans_complete(4,:)],'omitnan');
            % calc mk2 position
            R = mk_rot(:,:,mk_id) * rotx(theta);
            mk2 = mk4 - R(:,2)' * transducer_width;
            mk_trans_complete(2,:) = mean([mk2; mk_trans_complete(2,:)], 'omitnan');
            % calc mk1 position
            mk1 = mk2 + mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(1,:) = mean([mk1; mk_trans_complete(1,:)], 'omitnan');

        case 4
            % calc mk3 position
            mk3 = mk_trans_complete(mk_id,:) + mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(3,:) = mean([mk3; mk_trans_complete(3,:)], 'omitnan');
            % calc mk2 position
            R = mk_rot(:,:,mk_id) * rotx(theta);
            mk2 = mk_trans_complete(mk_id,:) - R(:,2)' * transducer_width;
            mk_trans_complete(2,:) = mean([mk2; mk_trans_complete(2,:)], 'omitnan');
            % calc mk1 position
            mk1 = mk2 + mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(1,:) = mean([mk1; mk_trans_complete(1,:)], 'omitnan');
    
    end
end

end

