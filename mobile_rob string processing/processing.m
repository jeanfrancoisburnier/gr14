%%processing data mobile rob
if not(exist('X_R'))
    X_R = cell(2,1);
    X = cell(2,1);
    Y_R = cell(2,1);
    Y = cell(2,1);
    T_R = cell(2,1);
    T = cell(2,1);
    i = 0;
end
close all
clearvars -except i A
i = i+1;
[X_R{2,i} X_R{1,i}]= extract_data('x_r.res');
[X{2,i} X{1,i}]= extract_data('x.res');
[Y_R{2,i} Y_R{1,i}]= extract_data('y_r.res');
[Y{2,i} Y{1,i}]= extract_data('y.res');
[T_R{2,i} T_R{1,i}]= extract_data('theta_r.res');
[T{2,i} T{1,i}]= extract_data('theta.res');

figure 
subplot(3,1,1)
plot(X_R{2,i},X_R{1,i});
hold on
plot(X{2,i},X{1,i});
title('X','Fontsize',12);
h_legend = legend('Real','Robot beliefs');

subplot(3,1,2)
plot(Y_R{2,i},Y_R{1,i});
hold on
plot(Y{2,i},Y{1,i});
title('Y','Fontsize',12);
h_legend = legend('Real','Robot beliefs');

subplot(3,1,3)
plot(T_R{2,i},T_R{1,i});
hold on
plot(T{2,i},T{1,i});
title('Theta','Fontsize',12);
h_legend = legend('Real','Robot beliefs');
