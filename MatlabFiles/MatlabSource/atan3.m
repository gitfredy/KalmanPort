function theta = atan3(y, x)
% atan2 that accepts symbolic expression

theta = 2*atan(y / ( sqrt(x^2+y^2) + x ));

end