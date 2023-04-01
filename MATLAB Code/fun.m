% Objective Function
function out = fun(X)
    x1 = X(:,1);
    x2 = X(:,2);
    out = x1.^2-x1.*x2+x2.^2+2.*x1+4.*x2+3;
end

