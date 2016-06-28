function Q = polyshift(P, a)
% shift the polynomial P so it has the same value at t+a that it had at t before

a = -a;
P = P(:)'; % ensure row vector

L = length(P);
N = L-1;

powers= cell(1, N+1);

if N>0
    powers{1} = 1;
    powers{2} = [1 a]; % (x+a)
    p2 = 1;
    r = 1;
    Q = zeros(size(P));
    Q(L) = P(L);
    Q(N:L) = Q(N:L) + P(N)*powers{2};
    for k=2:N
        Ck = conv(powers{p2+1}, powers{r+1}, 'full');
        ind = L-length(Ck)+1:L;
        Q(ind) = Q(ind) + P(L-k)*Ck;
        powers{k+1} = Ck;
        if (p2==r)
            p2 = 2*p2;
            r = 1;
        else
            r = r+1;
        end
    end % for-loop
else
    Q = P;
end

end %% polyshift
