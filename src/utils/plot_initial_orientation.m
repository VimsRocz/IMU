function plot_initial_orientation(out_dir, R_bn)
%PLOT_INITIAL_ORIENTATION Generate a simple 3-D axis plot of orientation.
%   PLOT_INITIAL_ORIENTATION(DIR, R_BN) saves a PDF figure showing the NED
%   axes transformed by R_BN.

    figure('Visible','off');
    quiver3(0,0,0,1,0,0,'r','LineWidth',1.5); hold on;
    quiver3(0,0,0,0,1,0,'g','LineWidth',1.5);
    quiver3(0,0,0,0,0,1,'b','LineWidth',1.5);
    T = R_bn';
    quiver3(0,0,0,T(1,1),T(1,2),T(1,3),'--r');
    quiver3(0,0,0,T(2,1),T(2,2),T(2,3),'--g');
    quiver3(0,0,0,T(3,1),T(3,2),T(3,3),'--b');
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    legend({'N','E','D','b_x','b_y','b_z'},'Location','best');
    title('Initial Orientation');
    pdf_file = fullfile(out_dir, 'initial_orientation.pdf');
    print('-dpdf', pdf_file);
    close(gcf);
end
