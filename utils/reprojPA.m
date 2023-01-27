% *************************************************************************
% reproject each pixel PA image to ECM image frame
% :param ecm: ecm image
% :param pa: pa image
% :param T_ecm_pa: transformation from ecm frame to pa frame
% :param cam_intri: camera intrinsic matrix
% :return pa_reproj: per-pixel reprojection of pa image under ecm frame
% *************************************************************************
function [pa_reproj] = reprojPA(ecm, pa, T_ecm_pa, cam_intri)

res_x = 0.253*1e-3;                 % lateral resolution [m/pix]
res_z = 0.053*1e-3;                           % axial resolution [m/pix]
pa_x = ((1:size(pa,2)) - size(pa,2)/2) * res_x;              % lateral fov, origin at top left [m]
pa_z = (1:size(pa,1)) * res_z;              % axial fov, origin at top left [m]

pa_reproj = nan(size(ecm,1:2));
for row = 1:size(pa, 1)
    for col = 1:size(pa,2)
        pix_pa_xyz = [pa_x(col); 0; pa_z(row); 1];
        pix_ecm_xyz = T_ecm_pa * pix_pa_xyz;
        pix_ecm_uv = cam_intri * pix_ecm_xyz(1:3);
        pix_ecm_uv = pix_ecm_uv./pix_ecm_uv(end);

        pix_ecm_uv(pix_ecm_uv <= 0) = 1;
        pix_ecm_v = min(ceil(pix_ecm_uv(1)), size(ecm,2));
        pix_ecm_u = min(ceil(pix_ecm_uv(2)), size(ecm,1));
        pa_reproj(pix_ecm_u, pix_ecm_v) = pa(row, col);
    end
end

end

