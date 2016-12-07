
corners_x = [-1.000 1.000;-0.850 0.850; -0.850 -0.350; 0.330 0.650; 0.330 0.650;-0.850 -0.350;-0.350 0.350; -0.350 0.050; -0.650 -0.350; -0.350 0.350];
corners_y = [-1.500 1.500;-1.350 1.350; 0.680 1; 0.850 1.350; -1.350 -0.850 ;-1 -0.680;0.150 0.550 ; -0.150 0.150 ; -0.250 0.250 ; -0.550 -0.150  ];
nbr_obs = size(corners_x);

figure


x_size = 0.850
y_size = 1.350
x = -x_size:0.1:x_size
y = -y_size:0.1:y_size

% Horizontal grid 
for k = 1:length(y)
  line([x(1) x(end)], [y(k) y(k)])
end

% Vertical grid
for k = 1:length(x)
  line([x(k) x(k)], [y(1) y(end)])
end

hold on 
for n=0:(nbr_obs(1)-1)
    if n == 0
        rectangle('Position',[corners_x(1+n,1) corners_y(1+n,1) corners_x(1+n,2)-corners_x(1+n,1) corners_y(1+n,2)-corners_y(1+n,1)],'LineWidth',2,'EdgeColor','k')
    end
    if n > 0
        rectangle('Position',[corners_x(1+n,1) corners_y(1+n,1) corners_x(1+n,2)-corners_x(1+n,1) corners_y(1+n,2)-corners_y(1+n,1)],'LineWidth',2)
    end
end
axis([-1.1 1.1 -1.6 1.6])
%axis('equal')
