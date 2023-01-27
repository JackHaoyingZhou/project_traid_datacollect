% *************************************************************************
% reproject four verticies of PA image to ECM image frame
% :param ecm: ecm image
% :param pa: pa image
% :param T_ecm_pa: transformation from ecm frame to pa frame
% :param cam_intri: camera intrinsic matrix
% :return pa_vert_reproj: verticies of pa image under ecm frame
% *************************************************************************
function [pa_vert_reproj] = reprojPAVert(ecm, pa, T_ecm_pa, cam_intri)

res_x = 0.253*1e-3;                 % lateral resolution [m/pix]
res_z = 0.053*1e-3;                 % axial resolution [m/pix]

pa_vert = [-size(pa,2)/2*res_x, size(pa,2)/2*res_x, -size(pa,2)/2*res_x, size(pa,2)/2*res_x;    % lateral (x)
    0, 0, size(pa,1)*res_z, size(pa,1)*res_z];   % axial (z)
pa_vert_reproj = zeros(size(pa_vert));

for v = 1:4
    pix_pa_xyz = [pa_vert(1,v); 0; pa_vert(2,v); 1];
    pix_ecm_xyz = T_ecm_pa * pix_pa_xyz;
    pix_ecm_uv = cam_intri * pix_ecm_xyz(1:3);
    pix_ecm_uv = pix_ecm_uv./pix_ecm_uv(end);

    pix_ecm_uv(pix_ecm_uv <= 0) = 1;
    pix_ecm_v = min(ceil(pix_ecm_uv(1)), size(ecm,2));
    pix_ecm_u = min(ceil(pix_ecm_uv(2)), size(ecm,1));
    pa_vert_reproj(:, v) = [pix_ecm_v, pix_ecm_u];
end

end

