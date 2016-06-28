function xn = normc(x)
    mag = sqrt(sum(x.^2, 1));
    xn = bsxfun(@rdivide, x, mag);
end