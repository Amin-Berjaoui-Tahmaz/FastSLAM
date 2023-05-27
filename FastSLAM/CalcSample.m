function [sample] = CalcSample(b)


r = -1 +rand(1,12)*(2);

sample=(b/6)*sum(r);

end

