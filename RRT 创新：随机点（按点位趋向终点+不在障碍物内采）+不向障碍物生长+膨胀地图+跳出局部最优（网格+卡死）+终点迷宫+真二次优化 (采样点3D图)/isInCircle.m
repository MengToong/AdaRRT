function isInside = isInCircle(center, point, radius)
    distance = sqrt((center(1) - point(1))^2 + (center(2) - point(2))^2);
    isInside = distance <= radius;
end

