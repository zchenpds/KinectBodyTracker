data = data(3:end, :);
data = data(data.isFollowing == 1, :);

t = (data.tRW - data.tRW(1))/1000;
tK = (data.tK - data.tK(1))/1000;

dt = [t(2:end) - t(1:end-1); 0];
dtK = [tK(2:end) - tK(1:end-1); 0];

subplot(3, 1, 1);
h11 = plot(t, data.vD, 'b'); 
hold on; 
h12 = plot(t, data.v, 'r'); 
hold off
grid on
ylabel("velotiy (m/s)");
xlabel("Time (s)");
legend([h11, h12], "Desired velocity", "Actual velocity")

subplot(3, 1, 2);
h21 = plot(t(dt~=0), 1./dt(dt~=0), 'g'); 
hold on; 
h22 = plot(t(dtK~=0), 1./dtK(dtK~=0), 'b'); 
% h4 = plot(t, ((data.xVm - data.xVmG).^2 + (data.yVm - data.yVmG).^2).^0.5, 'b'); 
legend([h21, h22], "f_R_W", "f_K_W")
hold off
ylabel("frequency (Hz)")

subplot(3, 1, 3);
h31 = plot(t, data.x - data.x(1), 'g'); 
hold on; 
h32 = plot(t, data.y - data.y(1), 'r'); 
xlabel("Time (s)");
ylabel("position coordinates (m)")
legend([h31, h32], "x", "y")
%subplot(2, 2, 2);



