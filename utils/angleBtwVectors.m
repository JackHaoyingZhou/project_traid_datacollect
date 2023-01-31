function [theta] = angleBtwVectors(u, v)

angle = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
theta = real(acosd(angle));

end

