function perp = find_perpendicular_to_line(pt, v1, v2)
    % Compute distance
    ptv2 = v2 - pt;
    ptv1 = v1 - pt;
    perp = pt + dot(ptv2, ptv1) / dot(ptv1, ptv1) * dot(ptv2, ptv1);
end