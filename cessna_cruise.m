clear; clc

ac = Aircraft();

ac.mass.set_mass_properties(767,1285,1824,2666,0,0,0,[2.11 0 1.26]);
ac.mass.set_fuel_capacity(144,1157);
ac.mass.set_fuel_mass(90);

ac.geometry.wing_area = 16.17;
ac.geometry.wing_span = 10.98;
ac.geometry.mean_aerodynamic_chord = 1.5;
ac.geometry.ref_area = 16.17;
ac.geometry.ref_span = 10.98;
ac.geometry.ref_chord = 1.5;

ac.aero.set_lookup(c172datcom("C:\Users\lucas\Documents\GitHub\aircraft-flight-simulator\AircraftDesign_Flightsim\Matlab\Example\Cessna172\cessna.out"));

cfg = ac.get_configurator();
cfg.add_control_surface('name',"aileron",'surface_type',"aileron",'classification',"primary",'axis',[1 0 0], ...
    'max_deflection',20,'min_deflection',-20,'dCl',0,'dCm',0,'dCn',0);
cfg.add_control_surface('name',"elevator",'surface_type',"elevator",'classification',"primary",'axis',[0 1 0], ...
    'max_deflection',28,'min_deflection',-26,'dCl',0.3,'dCm',-1.2,'dCn',0);
cfg.add_control_surface('name',"rudder",'surface_type',"rudder",'classification',"primary",'axis',[0 0 1], ...
    'max_deflection',30,'min_deflection',-30,'dCl',0,'dCm',0,'dCn',-0.075);

cfg.add_propulsive_element('name',"engine",'element_type','propeller','max_output',134228, ...
    'position',[1.68 0 1.26],'direction',[1 0 0],'fuel_rate',0,'thrust_model',[]);

disk_loading_factor = 0.05;
induced_velocity_factor = 0.1;
efficiency = 0.80;
Ct_coeff = [0.1, -0.05];
Cp_coeff = [0.05, 0.02];
diameter = 1.905; % meters
pitch = 1.219; % meters
num_blades = 2;
cfg.aircraft.propulsive_elements{1,1}.set_propeller_params(diameter,pitch,num_blades,efficiency,Ct_coeff,Cp_coeff,disk_loading_factor,induced_velocity_factor)

n_cs = numel(ac.control_surfaces);
n_pe = numel(ac.propulsive_elements);
n_total = n_cs + n_pe;

alt_cruise  = 1000;
mach_cruise = 0.14;
dur_cruise  = 120;

[~, a, ~, ~] = atmosisa(alt_cruise);
V_cruise = mach_cruise * a;

solver = ac.get_trim_solver();
solver.trim_tolerance = 1e-3;
solver.max_iterations = 15000;
solver.use_fmincon = true;
solver.initial_guess = [deg2rad(2); deg2rad(-1); 0.5];

[x_trim, u_trim, converged] = solver.solve_cruise_trim(alt_cruise, mach_cruise);

if ~converged
    x_trim = zeros(12,1);
    x_trim(3) = -alt_cruise;
    x_trim(4) = V_cruise;
    u_trim = zeros(n_total,1);
end

N_points = 0;
time_vector = linspace(0, dur_cruise, max(N_points,2));

state_ref   = repmat(x_trim, 1, numel(time_vector));
control_ref = repmat(u_trim, 1, numel(time_vector));

Initialpos = x_trim(1:3)';
InitialVel = x_trim(4:6)';
InitialOri = x_trim(7:9)';
InitialRot = x_trim(10:12)';

mission = struct();
mission.waypoints = struct('name',"Cruise",'type',"cruise",'altitude',alt_cruise,'mach',mach_cruise,'duration',dur_cruise, ...
    'state',x_trim,'controls',u_trim,'converged',converged,'velocity',V_cruise);
mission.timeline = struct('phase',"Cruise",'t_start',0,'duration',dur_cruise,'wp_start',1,'wp_end',1);
mission.time_vector = time_vector;
mission.altitude_profile = alt_cruise * ones(size(time_vector));
mission.velocity_profile = V_cruise * ones(size(time_vector));
mission.state_profile = state_ref;
mission.control_profile = control_ref;
mission.phase_profile = ones(size(time_vector));
mission.total_duration = dur_cruise;

control_input_data = [time_vector(:), control_ref.'];
sim_stop_time = dur_cruise;

assignin('base','ac',ac);
assignin('base','mission',mission);
assignin('base','waypoints',mission.waypoints);
assignin('base','timeline',mission.timeline);
assignin('base','initial_state',x_trim);
assignin('base','initial_controls',u_trim);
assignin('base','n_cs',n_cs);
assignin('base','n_pe',n_pe);
assignin('base','n_total',n_total);
assignin('base','control_input_data',control_input_data);
assignin('base','sim_stop_time',sim_stop_time);
assignin('base','autopilot',[]);
assignin('base','autopilot_enabled',0);
assignin('base','autopilot_mode',"off");
