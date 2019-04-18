clc
diffTrigger = diff(dataKinect.trigger);
i = 1+find(diffTrigger~=0);
pf = polyfit(dataKinect.tOW(i), dataKinect.tO(i), 1);
y = pf * [dataKinect.tO(i)'; ones(1, length(i))];
plot(i, -dataKinect.tO(i) + dataKinect.tOW(i) * pf(1) + pf(2))