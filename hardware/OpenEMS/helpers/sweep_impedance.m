

Zts = linspace(0,200, 1000);
s11s = zeros(size(Zts));
for k=1:length(Zts)

    s11s(k) = S11_eff(S_ant, Zts(k), 50);

end


plot(Zts, s11s);