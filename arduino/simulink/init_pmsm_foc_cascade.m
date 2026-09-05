%% PMSM FOC cascade-control model parameters
% The controller topology follows the supplied reference diagram.
% Motor and load values below are simulation placeholders (UNVERIFIED) and
% must be replaced with measured or datasheet values before plant conclusions.

Ts_current = 1.0e-4;
Ts_speed = Ts_current;
Ts_position = Ts_current;
Vdc = 12.0;

Kp_position = 8.0;
speed_ref_limit_rad_s = 80.0;

Kp_speed = 0.20;
Ki_speed = 4.0;
iq_ref_limit_a = 8.0;

Kp_id = 1.5;
Ki_id = 300.0;
Kp_iq = 1.5;
Ki_iq = 300.0;
voltage_limit_v = Vdc / sqrt(3.0);

% PMSM average-model placeholders (UNVERIFIED).
pmsm_pole_pairs = 7;
pmsm_Rs_ohm = 0.30;
pmsm_Ld_h = 2.0e-4;
pmsm_Lq_h = 2.0e-4;
pmsm_flux_wb = 6.0e-3;
pmsm_J_kg_m2 = 2.0e-5;
pmsm_B_nms = 1.0e-5;

position_step_rad = 2.0;
position_step_time_s = 0.02;
load_torque_nm = 0.0;
simulation_stop_time_s = 0.25;

