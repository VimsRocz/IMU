function plot_initial_orientation(R_bn, out_dir)
%PLOT_INITIAL_ORIENTATION  Visualise the body axes in NED.
%   PLOT_INITIAL_ORIENTATION(R_BN, OUT_DIR) draws a simple 3‑D plot of the
%   body axes defined by the rotation matrix and saves it as
%   ``initial_orientation.pdf`` in OUT_DIR.

    fig = figure('Visible','off'); %#ok<LFIG>
    quiver3(0,0,0,1,0,0,'r','LineWidth',1.5); hold on;
    quiver3(0,0,0,0,1,0,'g','LineWidth',1.5);
    quiver3(0,0,0,0,0,1,'b','LineWidth',1.5);
    A = R_bn';
    quiver3(0,0,0,A(1,1),A(1,2),A(1,3),'--r');
    quiver3(0,0,0,A(2,1),A(2,2),A(2,3),'--g');
    quiver3(0,0,0,A(3,1),A(3,2),A(3,3),'--b');
    xlabel('North'); ylabel('East'); zlabel('Down');
    legend({'N','E','D','Body X','Body Y','Body Z'},'Location','best');
    grid on; axis equal; view(35,20);
    file = fullfile(out_dir,'initial_orientation.pdf');
    print(fig, file, '-dpdf');
    close(fig);
end
