% x = logsout.getElement('local_pts').Values.Data(:,1);
% y = logsout.getElement('local_pts').Values.Data(:,2);
% plot(x, y, 'o-'); axis equal; grid on;
% title('Local Waypoints');
% xlabel('x [m]'); ylabel('y [m]');



clc;
% close all;
% 
% % --- Local Plot ---
% figure(1)
% x = local_pts(:,1);
% y = local_pts(:,2);
% plot(x, y, 'o-');
% axis equal; grid on;
% title('Local Waypoints');
% xlabel('x [m]'); ylabel('y [m]');
% 
% % --- Global Plot ---
% figure(2)
% xg = global_pts(:,1);
% yg = global_pts(:,2);
% plot(xg, yg, 'o-');
% axis equal; grid on;
% title('Global Waypoints');
% xlabel('X [m]'); ylabel('Y [m]');


xg = global_pts(:,1);
yg = global_pts(:,2);
plot(xg, yg, 'o-'); axis equal; grid on;
title('Global Waypoints'); xlabel('X [m]'); ylabel('Y [m]');