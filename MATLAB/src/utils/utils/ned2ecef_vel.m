function v_ecef = ned2ecef_vel(v_ned, r_ecef, C_ne)
%NED2ECEF_VEL Convert NED velocity to ECEF including Earth rotation term.
%   v_ecef = NED2ECEF_VEL(v_ned, r_ecef, C_ne)
%   Inputs:
%     v_ned : Nx3 velocity in NED frame [m/s]
%     r_ecef: Nx3 ECEF position [m]
%     C_ne  : 3x3xN rotation from ECEF to NED for each sample (C_ne)
%             i.e. C_en = C_ne'
%   Output:
%     v_ecef: Nx3 velocity in ECEF frame [m/s]
%
%   Computes v_e = C_en * v_ned + (omega_ie x r_e).
%   This mirrors the Python implementation and avoids large offsets.

omega_ie = 7.2921150e-5; % rad/s
N = size(v_ned,1);
v_ecef = zeros(N,3);
for k = 1:N
    C_en = C_ne(:,:,k)';
    cross_term = cross([0,0,omega_ie], r_ecef(k,:));
    v_ecef(k,:) = (C_en * v_ned(k,:).').' + cross_term;
end
end

