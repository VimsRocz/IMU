function plot_initial_orientation(R_bn, filename)
%PLOT_INITIAL_ORIENTATION  Plot body and NED axes for the estimated attitude.
%   PLOT_INITIAL_ORIENTATION(R_BN, FILENAME) creates a simple 3-D quiver plot
%   comparing the NED frame and the body frame rotated by R_BN. The figure is
%   saved to the specified PDF file.

    figure('Visible','off');
    origin = [0 0 0];
    quiver3(origin,origin,origin, [1 0 0], [0 1 0], [0 0 1], 0,'k','LineWidth',2,'DisplayName','NED');
    hold on;
    axes_body = R_bn' * eye(3); % body axes in NED
    quiver3(origin,origin,origin, axes_body(1,:), axes_body(2,:), axes_body(3,:), 0,'r','LineWidth',2,'DisplayName','Body');
    hold off; grid on; axis equal;
    xlabel('North'); ylabel('East'); zlabel('Down');
    legend('Location','best');
    title('Initial Orientation');
    if nargin > 1 && ~isempty(filename)
        print(gcf, filename, '-dpdf');
    end
    close(gcf);
end
