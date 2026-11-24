clear; clc; close all;
innerLoopHz = 200.0;
dt          = 1.0 / innerLoopHz; 
t_final     = 40.0;         
time_vec    = 0:dt:t_final;
N           = length(time_vec);

mass = 1.2;
g    = 9.81;
T_max = 80.0;     
maxLeanAngle  = deg2rad(80.0);   
hoverThrottle = 1000 + ((mass * g) / T_max) * 1000; 

T_MIN = 1000; T_IDLE = 1150; T_BOOST = 1850;
target_X = 0.0;
target_Y = 0.0;
target_Z = 10.0;
override_pitch = false;

L    = 0.30;      
Ixx = 0.06; Iyy = 0.06; Izz = 0.005;
I_matrix = diag([Ixx, Iyy, Izz]);
invI     = inv(I_matrix);
pos   = [0; 0; 0];      
vel   = [0; 0; 0];      
euler = [0; 0; 0];      
omega = [0; 0; 0];      
gimbal_x = 0; 
gimbal_y = 0; 
max_tvc_angle  = deg2rad(15); 
servo_deadband = deg2rad(0.3); 
active_cmd_x   = 0;
active_cmd_y   = 0;
delay_steps  = max(1, round(0.010 / dt)); 
euler_buffer = zeros(3, delay_steps);

pid_z     = struct('kp', 1.5,  'ki', 0.05, 'kd', 0.8,  'limit', 4.0, 'i_limit', 1.5, 'int', 0, 'prev_in', 0, 'deriv_filt', 0);
pid_vz    = struct('kp', 30.0, 'ki', 15.0, 'kd', 5.0,  'limit', 400.0, 'i_limit', 50.0, 'int', 0, 'prev_in', 0, 'deriv_filt', 0);
pid_x     = struct('kp', 0.25, 'ki', 0.0,  'kd', 0.45, 'limit', 2.0, 'i_limit', 0.0, 'int', 0, 'prev_in', 0, 'deriv_filt', 0);
pid_vx    = struct('kp', 0.20, 'ki', 0.0,  'kd', 0.40, 'limit', maxLeanAngle, 'i_limit', 0.0, 'int', 0, 'prev_in', 0, 'deriv_filt', 0);
pid_theta = struct('kp', 0.90, 'ki', 0.01, 'kd', 0.70, 'limit', max_tvc_angle, 'i_limit', max_tvc_angle/4, 'int', 0, 'prev_in', 0, 'deriv_filt', 0);
pid_y     = struct('kp', 0.25, 'ki', 0.0,  'kd', 0.45, 'limit', 2.0, 'i_limit', 0.0, 'int', 0, 'prev_in', 0, 'deriv_filt', 0);
pid_vy    = struct('kp', -0.20,'ki', 0.0,  'kd', -0.40,'limit', maxLeanAngle, 'i_limit', 0.0, 'int', 0, 'prev_in', 0, 'deriv_filt', 0);
pid_phi   = struct('kp', 0.90, 'ki', 0.01, 'kd', 0.70, 'limit', max_tvc_angle, 'i_limit', max_tvc_angle/4, 'int', 0, 'prev_in', 0, 'deriv_filt', 0);
log_pos      = zeros(3, N);
log_euler    = zeros(3, N);
log_servo    = zeros(2, N);
log_throttle = zeros(1, N); 
motor_killed = false;

for k = 1:N
    euler_buffer = [euler, euler_buffer(:, 1:end-1)];
    delayed_euler = euler_buffer(:, end);
    
    if time_vec(k) < 0.2 
        throttle = T_BOOST;
        target_theta = 0; target_phi = 0;
        raw_cmd_x = 0; raw_cmd_y = 0;
    else
        if pos(3) >= 95.0
            motor_killed = true;
        end
        
        if motor_killed
            throttle = T_MIN; 
            target_theta = 0; target_phi = 0;
            raw_cmd_x = 0; raw_cmd_y = 0;
        else
            t_curr = time_vec(k);
            if t_curr < 4.0
                target_X = 0; target_Z = 10;
                override_pitch = false;
            elseif t_curr < 5.5
                target_Z = 10;
                override_pitch = true;
            else
                if override_pitch
                    target_X = pos(1) + 20.0; 
                    override_pitch = false;
                end
                target_Z = 10;
            end
            
            [target_vz, pid_z] = computePID(target_Z, pos(3), dt, pid_z);
            [throttle_offset, pid_vz] = computePID(target_vz, vel(3), dt, pid_vz);
            
            throttle = round(max(min(hoverThrottle + throttle_offset, 1900), 1100));
            
            if override_pitch
                target_theta = deg2rad(80.0); 
                target_phi   = 0;
                pid_x.int = 0; pid_vx.int = 0; 
            else
                [target_vx, pid_x] = computePID(target_X, pos(1), dt, pid_x);
                [target_theta, pid_vx] = computePID(target_vx, vel(1), dt, pid_vx);
                target_theta = max(min(target_theta, maxLeanAngle), -maxLeanAngle);
                
                [target_vy, pid_y] = computePID(target_Y, pos(2), dt, pid_y);
                [target_phi, pid_vy] = computePID(target_vy, vel(2), dt, pid_vy); 
                target_phi = max(min(target_phi, maxLeanAngle), -maxLeanAngle);
            end
            
            noisy_theta = delayed_euler(2) + deg2rad(randn() * 0.5);
            noisy_phi   = delayed_euler(1) + deg2rad(randn() * 0.5);
            
            [raw_cmd_x, pid_theta] = computePID(target_theta, noisy_theta, dt, pid_theta);
            [raw_cmd_y, pid_phi]   = computePID(target_phi, noisy_phi, dt, pid_phi);
        end
    end
    if abs(raw_cmd_x - active_cmd_x) > servo_deadband
        active_cmd_x = raw_cmd_x;
    end
    if abs(raw_cmd_y - active_cmd_y) > servo_deadband
        active_cmd_y = raw_cmd_y;
    end
    gimbal_x = (0.75 * gimbal_x) + (0.25 * active_cmd_x);
    gimbal_y = (0.75 * gimbal_y) + (0.25 * active_cmd_y);
    T = max(0, (throttle - 1000) / 1000.0) * T_max;
    
    F_bx = -T * sin(gimbal_x) * cos(gimbal_y);
    F_by =  T * sin(gimbal_y) * cos(gimbal_x);
    F_bz =  T * cos(gimbal_x) * cos(gimbal_y);
    F_body = [F_bx; F_by; F_bz];
    
    tau_x =  L * F_by; 
    tau_y = -L * F_bx;
    tau_z = -0.01 * omega(3); 
    tau_body = [tau_x; tau_y; tau_z];
    
    omega_dot = invI * (tau_body - cross(omega, I_matrix * omega));
    omega = omega + omega_dot * dt;
    
    p = omega(1); q = omega(2); r = omega(3);
    phi = euler(1); theta = euler(2); psi = euler(3);
    
    phi_dot   = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta);
    theta_dot = q * cos(phi) - r * sin(phi);
    psi_dot   = (q * sin(phi) + r * cos(phi)) / cos(theta);
    
    euler = euler + [phi_dot; theta_dot; psi_dot] * dt;
    
    Rx = [1, 0, 0; 0, cos(euler(1)), -sin(euler(1)); 0, sin(euler(1)), cos(euler(1))];
    Ry = [cos(euler(2)), 0, sin(euler(2)); 0, 1, 0; -sin(euler(2)), 0, cos(euler(2))];
    Rz = [cos(euler(3)), -sin(euler(3)), 0; sin(euler(3)), cos(euler(3)), 0; 0, 0, 1];
    R_earth_body = Rz * Ry * Rx; 
    
    accel_earth = (R_earth_body * F_body) / mass - [0; 0; g];
    
    wind_accel = [randn() * 0.8; randn() * 0.8; randn() * 0.1];
    accel_earth = accel_earth + wind_accel;
    
    vel = vel + accel_earth * dt;
    pos = pos + vel * dt;
    
    if pos(3) <= 0 && k > 100
        pos(3) = 0; vel = [0;0;0]; omega = [0;0;0]; 
        euler = [0;0;0];
    end
    log_pos(:, k)   = pos;
    log_euler(:, k) = euler;
    log_servo(:, k) = [gimbal_x; gimbal_y];
    log_throttle(k) = throttle; 
end

figure('Name', '6-DOF Telemetry', 'Position', [50, 50, 600, 950]);
subplot(4,1,1);
line_z = animatedline('Color', 'b', 'LineWidth', 2);
grid on; hold on;
yline(target_Z, 'r--', 'Target Z'); ylabel('Altitude (m)'); title('Z-Axis Navigation');
xlim([0 t_final]); ylim([0 120]);
subplot(4,1,2);
line_x = animatedline('Color', 'm', 'LineWidth', 2);
line_y = animatedline('Color', 'c', 'LineWidth', 2);
grid on; hold on;
yline(target_X, 'm--', 'Target X'); yline(target_Y, 'c--', 'Target Y');
ylabel('Position (m)'); legend('X Pos', 'Y Pos', 'Location', 'best'); title('X/Y Navigation');
xlim([0 t_final]); ylim([0 120]);
subplot(4,1,3);
line_roll = animatedline('Color', 'g', 'LineWidth', 1.5);
line_pitch = animatedline('Color', 'r', 'LineWidth', 1.5);
grid on; hold on;
ylabel('Degrees'); legend('Roll (\phi)', 'Pitch (\theta)', 'Location', 'best'); title('Attitude Dynamics');
xlim([0 t_final]); ylim([-15 15]); 
subplot(4,1,4);
line_esc = animatedline('Color', 'k', 'LineWidth', 1.5);
grid on; hold on;
yline(1000, 'r--', 'Min (1000)'); yline(2000, 'r--', 'Max (2000)');
ylabel('Signal (\mus)'); legend('FC to ESC PWM', 'Location', 'best'); title('ESC Signal Output');
xlim([0 t_final]); ylim([900 2100]);
xlabel('Time (s)');

disp('Starting Live 6-DOF Telemetry & 3D Rendering...');
figure('Name', '6-DOF Flight Viewer', 'Position', [700, 100, 900, 800]);
ax = axes('XLim', [-10 120], 'YLim', [-10 120], 'ZLim', [0 120]);
view(45, 30); grid on; hold on;
xlabel('X Position (m)'); ylabel('Y Position (m)'); zlabel('Altitude (m)');
plot3(target_X, target_Y, target_Z, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
trail = animatedline('Color', 'b', 'LineWidth', 1.5, 'MaximumNumPoints', 800);
fv = stlread('tvc.stl'); 
t = hgtransform('Parent', ax);
patch('Faces', fv.ConnectivityList, 'Vertices', fv.Points, ...
      'Parent', t, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', ...
      'FaceLighting', 'gouraud');
camlight('headlight');
for i = 1:8:length(time_vec)
    t_curr = time_vec(i);
    p_x = log_pos(1, i); p_y = log_pos(2, i); p_z = log_pos(3, i);
    phi = log_euler(1, i); theta = log_euler(2, i); psi = log_euler(3, i);
    
    addpoints(trail, p_x, p_y, p_z);
    
    addpoints(line_z, t_curr, p_z);
    addpoints(line_x, t_curr, p_x);
    addpoints(line_y, t_curr, p_y);
    addpoints(line_roll, t_curr, rad2deg(phi));
    addpoints(line_pitch, t_curr, rad2deg(theta));
    addpoints(line_esc, t_curr, log_throttle(i));
    
    T_mat = makehgtform('translate', [p_x, p_y, p_z]);
    R_yaw   = makehgtform('zrotate', psi);
    R_pitch = makehgtform('yrotate', theta);
    R_roll  = makehgtform('xrotate', phi);
    R_base = makehgtform('xrotate', pi/2);
    S_mat = makehgtform('scale', 0.005); 
    
    t.Matrix = T_mat * (R_yaw * R_pitch * R_roll) * R_base * S_mat; 
    drawnow; 
end
function [out, state] = computePID(sp, in, dt, state)
    err = sp - in;
    state.int = state.int + (err * dt);
    state.int = max(min(state.int, state.i_limit), -state.i_limit);
    deriv_raw = -(in - state.prev_in) / dt; 
    state.deriv_filt = (0.8 * state.deriv_filt) + (0.2 * deriv_raw);
    deriv = state.deriv_filt;
    state.prev_in = in;
    out = (state.kp * err) + (state.ki * state.int) + (state.kd * deriv);
    out = max(min(out, state.limit), -state.limit); 
end