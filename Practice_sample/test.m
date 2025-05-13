x = logsout.getElement('Local_Points').Values.Data(:,1);
y = logsout.getElement('Local_Points').Values.Data(:,2);
plot(x, y, 'o-'); axis equal; grid on;
title('Local Waypoints'); xlabel('x [m]'); ylabel('y [m]');