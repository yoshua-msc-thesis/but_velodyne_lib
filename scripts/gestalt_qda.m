function [polynoms, A] = gestalt_qda(descriptors, degree)

    descriptors(~all(~isnan(descriptors), 2),:)=[];
    N=size(descriptors, 1);
    D=size(descriptors, 2)/2;
    all_feats = [descriptors(:, 1:D); descriptors(:, D+1:2*D)];
    
    poly = [];
    for i=1:D
        feats = all_feats(:, i);
        [cdf, bins] = ecdf(feats);
        gauss_X = norminv(cdf);
        points = length(gauss_X);
        poly_i = polyfit(bins(2:points-1), gauss_X(2:points-1), degree);
        poly = [poly; poly_i];
        all_feats(:, i) = polyval(poly_i, all_feats(:, i));
    end
    
    src_feats = all_feats(1:N, :);
    trg_feats = all_feats(N+1:2*N, :);
    
    sigma_M = cov(src_feats-trg_feats);
    sigma_U = cov(src_feats-trg_feats(randperm(size(trg_feats, 1)), :));

    AtA = inv(sigma_M)-inv(sigma_U);
    [V, D] = eig(AtA);
    D(D<0) = 0.000001;
    AtA = V*D*inv(V);
    
    polynoms = poly;
    A = chol(AtA);
end
