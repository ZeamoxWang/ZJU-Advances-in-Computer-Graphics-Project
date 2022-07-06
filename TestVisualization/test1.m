load test1.mat;
f = fit([Ce,Cts],time_saving,'cubicinterp');
plot(f,[Ce,Cts],time_saving);
ztickformat('%.2f\%');
title('The Time-Saving Ratio Plot Between Parallel and Serial KD-Tree');
xlabel('C_e');
ylabel('C_{ts}');
