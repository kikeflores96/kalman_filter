function [I] = resample(q)
N=length(q);
qc = [0 cumsum(q)];
[~,I]=histc(sort(rand(N,1)),qc);
end

