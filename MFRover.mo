package RoverExample

  model ExampleScenario
  // parameters
    parameter Integer fidelity = 2
      annotation(choices(choice = 1 "low-fidelity", choice = 2 "high-fidelity"));     // select fidelity level
    Components.Rover rover(fidelity = fidelity) annotation(
      Placement(transformation(origin = {183, 27}, extent = {{-33, -33}, {33, 33}})));
    Components.Webserver webserver annotation(
      Placement(transformation(origin = {-87, 41}, extent = {{-29, -29}, {29, 29}})));
    Components.Controller controller annotation(
      Placement(transformation(origin = {41, 19}, extent = {{-35, -35}, {35, 35}})));
      
  equation
  
    
    connect(webserver.udp, controller.udp) annotation(
      Line(points = {{-49, 41}, {-4, 41}}, thickness = 0.5));
    
    connect(controller.sensor, rover.sensor) annotation(
      Line(points = {{-5, 13}, {-5, -36}, {226, -36}, {226, 34}}, thickness = 0.5));
    
    connect(controller.pwm, rover.pwm) annotation(
      Line(points = {{88, 34}, {140, 34}}, thickness = 0.5));
  
  annotation(
      Diagram(coordinateSystem(extent = {{-120, 80}, {240, -40}})),
  experiment(StartTime = 0, StopTime = 305.0, Tolerance = 1e-06, Interval = 0.02));
  
  end ExampleScenario;
  
// (redeclare model RoverModel = RoverExample.Components.RoverHighFidelity)

  package Components
    model Webserver
    // interaction of the user-making discrete turn commands on the website
      parameter Real sample_interval = 0.1;      // [sec] update rate for command
      parameter Real turn_interval = 7.0;       // [sec] interval between straight, left, right sequence
      parameter Real repeat_interval = 21.0;          // [sec] repeat sequence
      parameter Integer turn_interval_count = integer(turn_interval/sample_interval);       // [-] count for turn interval
      parameter Integer repeat_interval_count = integer(repeat_interval/sample_interval);               // [-] count for sample interval
      // output, turn signal
      Connectors.IntegerOutput turn;
      Connectors.UdpBus udp annotation(
        Placement(transformation(origin = {120, 20}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {130.8, 1}, extent = {{-28.8, -18}, {28.8, 18}})));
      // internal state for timer, time needs to be incorprated as state due to FMU sim environment
      discrete Integer timer_count(start=0);
    
    equation
    
      connect(udp.turn, turn);
      
    algorithm
// initialize after a loop, increase count
      when sample(0, sample_interval) then
        timer_count := timer_count+1; 
      end when;
// algorithm models turn commands
// turn command example: 0 (no turn) for (turn_interval) secs -> 1 (right turn) for (turn_interval) secs
// -> -1 (left turn) for (turn_interval) secs) -> 0 (no turn) for (turn_interval) secs
      if (timer_count <= 1*turn_interval_count) and (timer_count >= 0*turn_interval_count) then
        turn := 0;
        
      elseif (timer_count <= 2*turn_interval_count) and (timer_count > 1*turn_interval_count) then
        turn := 1;
        
      elseif (timer_count <= 3*turn_interval_count) and (timer_count > 2*turn_interval_count) then
        turn := 1;
        
      else
        turn := 0;
        
      end if;
// comment out: single turn simulation

      if timer_count >= repeat_interval_count then
        timer_count := 0;
      end if;
     
      annotation(
        Diagram,
        Icon(graphics = {Rectangle(origin = {0, 30}, extent = {{-80, 50}, {80, -50}}), Rectangle(origin = {0, 30}, extent = {{-72, 44}, {72, -44}}), Polygon(origin = {0, -60}, points = {{-64, 22}, {64, 22}, {80, -24}, {-82, -24}, {-64, 22}}), Polygon(origin = {2, -60}, points = {{-72, 28}, {68, 28}, {88, -30}, {-92, -30}, {-72, 28}}), Text(origin = {1, 30}, extent = {{-67, 38}, {67, -38}}, textString = "webserver")}),
        experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.02));
    end Webserver;
    
    model Controller                                  // a simple controller model of the Rover FSM
      // packages and functions
      import RoverExample.Constants;
      import RoverExample.Utils.eul2quat;
      import RoverExample.Utils.quat2rot;
      import RoverExample.Utils.quat2eul;
      // parameters
      parameter Real delta_max = 25.28*Constants.PI/180;        // [rad] max steering angle
      parameter Real delta_turn = 25.28*Constants.PI/180;       // [rad] turn steering angle
      parameter Real v_fwd = 1.0;                               // [m/s] nominal forward velocity
      parameter Real v_max = 15.0;                              // [m/s] max forward velocity
      parameter Real sample_interval = 0.1;                     // [sec] sampling time
      parameter Real turn_angle = 70*Constants.PI/180;                      // [rad] heading angle change for turn
      // input, sensor
      Connectors.SensorBus sensor annotation(
        Placement(transformation(origin = {-108, -16}, extent = {{-8, -5}, {8, 5}}), iconTransformation(origin = {-132.8, -17.25}, extent = {{-29.2, -18.25}, {29.2, 18.25}})));
      Connectors.RealInput x, y;
      Connectors.RealInput phi_gyro, theta_gyro, psi_gyro;
      Connectors.RealInput ax_acc, ay_acc, az_acc;
      Connectors.RealInput p_gyro, q_gyro, r_gyro;
      Connectors.RealInput mx_mag, my_mag, mz_mag;
      // input, udp
      Connectors.UdpBus udp annotation(
        Placement(transformation(origin = {-108, 4}, extent = {{-8, -5}, {8, 5}}), iconTransformation(origin = {-128, 61.5}, extent = {{-28, -17.5}, {28, 17.5}})));
      Connectors.IntegerInput turn;
      // output, pwm
      Connectors.PwmBus pwm annotation(
        Placement(transformation(origin = {-108, -38}, extent = {{-8, -5}, {8, 5}}), iconTransformation(origin = {133, 42.875}, extent = {{-29, -18.125}, {29, 18.125}})));
      Connectors.IntegerOutput pwm_steering, pwm_throttle;
      // state logic
      discrete Real d(start=0);                               // [m] distance from reference point
      discrete Real psi_change(start=0);                      // [rad] current turning angle
      discrete Integer s(start=0);                            // [-] state of controller
      discrete Real x_ref(start=0);                           // [m] reference x-coordinate
      discrete Real y_ref(start=0);                           // [m] reference y-coordinate
      discrete Real psi_ref(start=0);                         // [rad] reference heading
      discrete Real v(start=0);                               // [m/s] velocity
      discrete Real delta(start=0);                                                   // [rad] steering angle
      // filtering - AHRS, Madgwick complementary filter
      parameter Real alpha = 1.0;                             // [-] relative weight for magnetometer
      parameter Real beta = 0.1;                              // [-] weight for complementary filter
      discrete Real quaternion_filtered[4](start={1,0,0,0}, each fixed=false);  // [-] filtered quaternion
      discrete Real euler_filtered[3](start={0,0,0}, each fixed=false);         // [rad] filtered euler angle
      discrete Real psi_filtered(start=0);                                
// [rad] filtered heading
    // updated: improved complementary filter
    //discrete Real psi_pred(start=0);                      // [rad] predicted heading by integrating yaw rate
    equation
    
      connect(udp.turn, turn);
      
      connect(pwm.ch_0, pwm_throttle);
      connect(pwm.ch_1, pwm_steering);
      
      connect(sensor.x, x);
      connect(sensor.y, y);
      connect(sensor.phi, phi_gyro);
      connect(sensor.theta, theta_gyro);
      connect(sensor.psi, psi_gyro);
      connect(sensor.ax, ax_acc);
      connect(sensor.ay, ay_acc);
      connect(sensor.az, az_acc);
      connect(sensor.p, p_gyro);
      connect(sensor.q, q_gyro);
      connect(sensor.r, r_gyro);
      connect(sensor.mx, mx_mag);
      connect(sensor.my, my_mag);
      connect(sensor.mz, mz_mag);
    
    algorithm
      
      when sample(0, sample_interval) then
// translate output to pwm
        pwm_throttle := 1000 + integer(1000*v/v_max);
        pwm_steering := 1500 + integer(500*delta/delta_max);
// updated: improved complementary filter
//psi_pred := psi_filtered+r_gyro*sample_interval;
//psi_filtered := atan2(alpha*sin(psi_pred)+(1-alpha)*sin(psi_mag),alpha*cos(psi_pred)+(1-alpha)*cos(psi_mag));
        quaternion_filtered := MadgwickFusionStep(ax_acc, ay_acc, az_acc, p_gyro, q_gyro, r_gyro, mx_mag, my_mag, mz_mag, quaternion_filtered, alpha, beta, sample_interval);
           
        euler_filtered := quat2eul(quaternion_filtered);
        psi_filtered := euler_filtered[3];
    
        d := sqrt((x - x_ref)^2 + (y - y_ref)^2);
        psi_change := mod(psi_filtered - psi_ref + Constants.PI, 2*Constants.PI) - Constants.PI;
        
        if s==0 and turn < 0 then
          s := 1;
          x_ref := x;
          y_ref := y;
          psi_ref := psi_filtered;
          v := v_fwd;
          delta := 0;
        
        elseif s==0 and turn > 0 then
          s := 3;
          x_ref := x;
          y_ref := y;
          psi_ref := psi_filtered;
          v := v_fwd;
          delta := 0;
        
        elseif s==1 and d > 7 then
          s := 2;
          x_ref := x_ref;
          y_ref := y_ref;
          psi_ref := psi_ref;
          v := v_fwd;
          delta := delta_turn;
        
        elseif s==3 and d > 7 then
          s := 4;
          x_ref := x_ref;
          y_ref := y_ref;
          psi_ref := psi_ref;
          v := v_fwd;
          delta := -delta_turn;
     
        elseif s==2 and psi_change > turn_angle then
          s := 5;
          x_ref := x;
          y_ref := y;
          psi_ref := psi_filtered;
          v := v_fwd;
          delta := 0;
          
        elseif s==4 and psi_change < -turn_angle then
          s := 5;
          x_ref := x;
          y_ref := y;
          psi_ref := psi_filtered;
          v := v_fwd;
          delta := 0;
          
        elseif s==5 and d > 7 then
          s := 0;
          x_ref := x_ref;
          y_ref := y_ref;
          psi_ref := psi_ref;
          v := 0;
          delta := 0;
        end if;
    
      end when;
    
      annotation(
        Icon(graphics = {Rectangle(origin = {-30, -18}, extent = {{-50, 30}, {50, -30}}), Polygon(origin = {0, 52}, points = {{-80, -40}, {20, 40}, {80, 40}, {20, -40}, {-80, -40}}), Polygon(origin = {50, 22}, points = {{30, 70}, {30, 30}, {-30, -70}, {-30, -10}, {30, 70}}), Rectangle(origin = {-63, -19}, extent = {{-9, 9}, {9, -9}}), Rectangle(origin = {-30, -18}, extent = {{-10, 16}, {10, -16}}), Rectangle(origin = {-30, -9}, extent = {{-8, 3}, {8, -3}}), Rectangle(origin = {-30, -21}, extent = {{-8, 3}, {8, -3}}), Rectangle(origin = {-2, -18}, extent = {{-10, 16}, {10, -16}}), Rectangle(origin = {-2, -9}, extent = {{-8, 3}, {8, -3}}), Rectangle(origin = {-2, -21}, extent = {{-8, 3}, {8, -3}}), Ellipse(origin = {-38, 28}, extent = {{-10, 4}, {10, -4}}), Ellipse(origin = {4, 28}, extent = {{-10, 4}, {10, -4}}), Ellipse(origin = {13, 55}, extent = {{-9, 3}, {9, -3}}), Ellipse(origin = {24, 82}, extent = {{-6, 2}, {6, -2}}), Ellipse(origin = {50, 82}, extent = {{-6, 2}, {6, -2}}), Polygon(origin = {33, 10}, points = {{-5, -2}, {5, 10}, {5, 4}, {-5, -10}, {-5, -2}}), Polygon(origin = {61, 54}, points = {{1, 2}, {5, 8}, {5, 4}, {1, -2}, {1, 2}}), Text(origin = {1, -72}, extent = {{-97, 20}, {97, -20}}, textString = "controller"), Rectangle(origin = {-68, 69}, extent = {{-12, 15}, {12, -15}})}),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
    end Controller;
    
    model Rover
      // a multi-fidelity rover model
      
      // load packages
      import RoverExample.Constants;
      
      // setup fidelity level and load different fidelity rover model
      parameter Integer fidelity = 2; 
      RoverLowFidelity rover_3d;
      RoverHighFidelity rover_8d;
      
      // system parameters
      parameter Real sample_interval = 0.1;                     // [sec] pwm sample interval
      parameter Real delta_max = 25.28*Constants.d2r;           // [rad] max steering angle
      
      // targeted acoustic attack model and parameters
      parameter Real W = 0;                                     // [W] power of speaker
      parameter Real dist = 0.01;                               // [m] distance to speaker
      parameter Real psi_ac = 80.0*Constants.d2r;               // [rad] speaker direction
      parameter Real w_ac = 15.0002e+3*2*Constants.PI;          // [rad/s] acoustic attack frequency
      parameter Real epsilon = 0.0*Constants.d2r;               // [rad] misalignment of gyroscope, reference - 1deg
      parameter Real phi_0 = 30*Constants.d2r;                  // [rad] phase shift for acoustic noise compared to driving signal
      GyroAcousticAtk gyroatk(W=W, dist=dist, psi_ac=psi_ac, w_ac=w_ac, epsilon=epsilon, phi_0=phi_0);
      
      // input: two PWM channels
      // channel 1: pwm duty cycle for throttle (0: 0, v_max: 1)
      // channel 2: steering command (-1: -delta_max, 1: delta_max)
      Connectors.PwmBus pwm annotation(
        Placement(transformation(origin = {-124, -2}, extent = {{-22, -22}, {22, 22}}), iconTransformation(origin = {-129.4, 21.125}, extent = {{-28.6, -17.875}, {28.6, 17.875}})));
      Connectors.RealInput pwm_steering, pwm_throttle;
      
      // output
      // local coordinates, body velocity, body acceleration, roll/pitch angle, heading, roll/pitch/yaw rate, rollover signal
      Connectors.SensorBus sensor annotation(
        Placement(transformation(origin = {128.6, 2.375}, extent = {{-26.6, -16.625}, {26.6, 16.625}}), iconTransformation(origin = {129.6, 21.75}, extent = {{-27.6, -17.25}, {27.6, 17.25}})));
      Connectors.RealOutput x_meas, y_meas, z_meas;
      Connectors.RealOutput vx_meas, vy_meas, vz_meas;
      Connectors.RealOutput ax_meas, ay_meas, az_meas;
      Connectors.RealOutput phi_meas, theta_meas, psi_meas;
      Connectors.RealOutput p_meas, q_meas, r_meas;
      Connectors.RealOutput mx_meas, my_meas, mz_meas;
      Connectors.RealOutput rollover_detected;
      
    algorithm
// algorithm models pwm sampling of ESC/servo
      when sample(0, sample_interval) then
      
        if fidelity == 1 then
          rover_3d.D := (pwm_throttle - 1000)/1000;
          rover_3d.delta_cmd := delta_max*(pwm_steering - 1500)/500;
          rover_8d.D := 0;
          rover_8d.delta_cmd := 0;
          
        elseif fidelity == 2 then
          rover_3d.D := 0;
          rover_3d.delta_cmd := 0;
          rover_8d.D := (pwm_throttle - 1000)/1000;
          rover_8d.delta_cmd := delta_max*(pwm_steering - 1500)/500;
          
        end if;
      end when;
      
    equation
    
      connect(sensor.x, x_meas);
      connect(sensor.y, y_meas);
      connect(sensor.z, z_meas);
      connect(sensor.vx, vx_meas);
      connect(sensor.vy, vy_meas);
      connect(sensor.vz, vz_meas);
      connect(sensor.ax, ax_meas);
      connect(sensor.ay, ay_meas);
      connect(sensor.az, az_meas);
      connect(sensor.phi, phi_meas);
      connect(sensor.theta, theta_meas);
      connect(sensor.psi, psi_meas);
      connect(sensor.p, p_meas);
      connect(sensor.q, q_meas);
      connect(sensor.r, r_meas);
      connect(sensor.mx, mx_meas);
      connect(sensor.my, my_meas);
      connect(sensor.mz, mz_meas);
     
      connect(pwm.ch_0, pwm_throttle);
      connect(pwm.ch_1, pwm_steering);
// get outputs from the selected model
      if fidelity == 1 then
        x_meas = rover_3d.x;
        y_meas = rover_3d.y;
        z_meas = rover_3d.z;
        vx_meas = rover_3d.vx;
        vy_meas = rover_3d.vy;
        vz_meas = rover_3d.vz;
        ax_meas = rover_3d.ax-rover_3d.specific_g[1];
        ay_meas = rover_3d.ay-rover_3d.specific_g[2];
        az_meas = rover_3d.az-rover_3d.specific_g[3];
        phi_meas = rover_3d.phi;
        theta_meas = rover_3d.theta;
        psi_meas = mod(rover_3d.psi + Constants.PI, 2*Constants.PI) - Constants.PI;
        p_meas = rover_3d.p;
        q_meas = rover_3d.q;
        r_meas = rover_3d.r+gyroatk.omega_yaw_false;
        mx_meas = rover_3d.mx;
        my_meas = rover_3d.my;
        mz_meas = rover_3d.mz;
        rollover_detected = rover_3d.rollover_detected;
        
      elseif fidelity == 2 then
        x_meas = rover_8d.x;
        y_meas = rover_8d.y;
        z_meas = rover_8d.z;
        vx_meas = rover_8d.vx;
        vy_meas = rover_8d.vy;
        vz_meas = rover_8d.vz;
        ax_meas = rover_8d.ax-rover_8d.specific_g[1];
        ay_meas = rover_8d.ay-rover_8d.specific_g[2];
        az_meas = rover_8d.az-rover_8d.specific_g[3];
        phi_meas = rover_8d.phi;
        theta_meas = rover_8d.theta;
        psi_meas = mod(rover_8d.psi + Constants.PI, 2*Constants.PI) - Constants.PI;
        p_meas = rover_8d.p;
        q_meas = rover_8d.q;
        r_meas = rover_8d.r+gyroatk.omega_yaw_false;
        mx_meas = rover_8d.mx;
        my_meas = rover_8d.my;
        mz_meas = rover_8d.mz;
        rollover_detected = rover_8d.rollover_detected;
        
      end if;
        
    annotation(
        Icon(graphics = {Rectangle(extent = {{-60, 80}, {60, -80}}), Rectangle(origin = {-74, 60}, extent = {{-6, 20}, {6, -20}}), Rectangle(origin = {74, 60}, extent = {{-6, 20}, {6, -20}}), Rectangle(origin = {-74, -60}, extent = {{-6, 20}, {6, -20}}), Rectangle(origin = {74, -60}, extent = {{-6, 20}, {6, -20}}), Text(origin = {-3, 11}, extent = {{-55, 61}, {55, -61}}, textString = "rover")}),
        experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.02));
    
    end Rover;

    model RoverLowFidelity
      // low-fidelity rover model: emulate rover kinematics (without ground friction, motor/actuator model)
    
      // load packages
      import RoverExample.Constants;
      import RoverExample.Utils.clip;
      import RoverExample.Utils.eul2rot;
      
      // parameters
      parameter Real l_total = 0.278;           // [m] distance from rear axle to front axle
      parameter Real track_width = 0.234;       // [m] track width
      parameter Real cg_height = 0.064;         // [m] height to center of gravity
      parameter Real phi_t = 0;                 // [m] terrain angle
      parameter Real v_max = 15.0;              // [m/s] max forward velocity
      
      // calculated from parameters
      Real phi_f;                               // [rad] angle between the line from end of rie to cg and the ground
      Real length_to_tire;                      // [m] distance from cg to tire end
      
      // inputs
      discrete Real D(start=0);
      discrete Real delta_cmd(start=0);
      
      // states
      Real x(start=0,fixed=false), y(start=0,fixed=false), z(start=0,fixed=false);            // [m] position in inertial coordinates
      Real vx(start=0,fixed=false), vy(start=0,fixed=false), vz(start=0,fixed=false);         // [m/s] velocity in body coordinates
      Real ax(start=0,fixed=false), ay(start=0,fixed=false), az(start=0,fixed=false);         // [m/s^2] acceleration in body coordinates
      Real phi(start=0,fixed=false), theta(start=0,fixed=false), psi(start=0,fixed=false);    // [rad] rover attitude (inertial to body)
      Real p(start=0,fixed=false), q(start=0,fixed=false), r(start=0,fixed=false);            // [rad/s] angular velocity
      Real turn_radius(start=1000);                                                           // [m] turning radius
      Integer rollover_detected(start=0);                                                     // [-] rollover detection signal
      Real mx(start=0,fixed=false), my(start=0,fixed=false), mz(start=0,fixed=false);         // [T] magnetic field measured from magnetometer
      
      // internal states
      Real thr, delta;
      
      // accelerometer specific force compensation
      Real specific_g[3](start={0.0, 0.0, Constants.g}, each fixed=false);      // [m/s^2] gravity specific force
      Real C_n2b[3,3](start={{1, 0, 0},{0, 1, 0},{0, 0, 1}},each fixed=false);  // [-] coordinate transform from NED to body
      
      // load magnetometer model
      Magnetometer mag;
    
    equation
      thr = v_max*pre(D);
      delta = pre(delta_cmd);
    
      phi_f = atan2(cg_height, 0.5*track_width);
      length_to_tire = sqrt(cg_height^2 + (0.5*track_width)^2);
      
      der(x) = thr*cos(psi);
      der(y) = thr*sin(psi);
      der(z) = 0;
      vx = thr;
      der(vy) = 0;
      der(vz) = 0;
      ax = 0;
      ay = thr*thr/l_total*tan(-delta);
      az = 0;
      der(phi) = 0;
      der(theta) = 0;
      der(psi) = thr/l_total*tan(delta);
      p = 0;
      q = 0;
      r = thr/l_total*tan(delta);

      // rollover check
      when delta <> 0 then
        turn_radius = clip(abs(l_total/tan(delta)),0,1000);
      elsewhen delta == 0 then
        turn_radius = 1000;
      end when;
    
      when Constants.g*cos(phi_f+phi_t) <= (thr^2/turn_radius)*sin(phi_f) then
        rollover_detected = 1;
      elsewhen Constants.g*cos(phi_f+phi_t) > (thr^2/turn_radius)*sin(phi_f) then
        rollover_detected = 0;
      end when;
  
      // accelerometer specific force
      C_n2b = transpose(eul2rot({phi, theta, mod(psi + Constants.PI, 2*Constants.PI) - Constants.PI}));
      for i in 1:3 loop
        specific_g[i] = C_n2b[i,3]*Constants.g;
      end for;
  
      // magnetometer model inputs
      mag.phi = phi;
      mag.theta = theta;
      mag.psi = psi;
      mx = mag.b_earth[1];
      my = mag.b_earth[2];
      mz = mag.b_earth[3];
    
    end RoverLowFidelity;

    model RoverHighFidelity  
      // high-fidelity rover model: simulate rover dynamics (tire-ground interaction, motor/actuator model, chassis rigid body dynamics)
    
      // load packages
      import RoverExample.Constants;
      import RoverExample.Utils.clip;
      import RoverExample.Utils.eul2rot;
      
      // parameters for chassis, tire and motor dynamics
      parameter Real mass_total = 4.177;                    // [kg] rover mass
      parameter Real mass_wheel = 0.141;                    // [kg] rover tire mass
      parameter Real mass_unsprung_front = 8.0*mass_wheel;  // [kg] front unsprung mass
      parameter Real mass_unsprung_rear = 8.0*mass_wheel;   // [kg] rear unsprung mass
      parameter Real mass_sprung = mass_total-mass_unsprung_front-mass_unsprung_rear;
      parameter Real l_total = 0.275;                       // [m] distance from rear axle to front axle
      parameter Real l_front = 0.136;                       // [m] distance from cg to front axle
      parameter Real l_rear = 0.139;                        // [m] distance from cg to rear axle
      parameter Real tw = 0.234;                            // [m] rover trackwidth
      parameter Real r_tire = 0.056;                        // [m] rover tire radius
      parameter Real w_tire = 0.06;                         // [m] tire width
      parameter Real huf = 0.01;                            // [m] height of front unsprung mass cg from roll axis
      parameter Real hur = 0.01;                            // [m] height of rear unsprung mass cg from roll axis
      parameter Real hs = 0.06;                             // [m] height of rear sprung mass cg from roll axis
      parameter Real I_wheel = 1/2*mass_wheel*r_tire^2*3;       // [kg*m^2] rolling moment of inertia of wheels
      parameter Real I_zz = 1/12*mass_total*(l_total^2+tw^2);   // [kg*m^2] yaw moment of inertia
      parameter Real I_xx = 1/2*mass_total*(tw/2)^2;            // [kg*m^2] roll moment of inertia
      parameter Real c_aero = 0.0;              // [1/m] aerodynamic resistance coefficient
      parameter Real mu0 = 0.8;                 // [-] maximum friction scaling coefficient, [0.4 1.2]
      parameter Real As = 0.0;                  // [-] friction reduction factor
      parameter Real c_kappa = 4.0;             // [N] longitudinal stiffness
      parameter Real c_alpha = 20.0;            // [N/rad] lateral stiffness
      parameter Real Lrelx = 0.185;             // [m] longitudinal relaxation length
      parameter Real Lrely = 0.185;             // [m] lateral realaxation length
      parameter Real k_rllsp = 2.00;              // [N/rad] rolling stiffness
      parameter Real c_rllsp = 1.00;              // [N/(rad/s)] rolling damping
      parameter Real normal_lim = 1.55;           // [N] normal force limit for rolling over
      parameter Real kappa_max = 0.999;           // [-] upper bound of slip ratio
      parameter Real kappa_min = -0.999;          // [-] lower bound of slip ratio
      parameter Real alpha_max = Constants.PI/2;  // [rad] upper bound of slip angle
      parameter Real alpha_min = -Constants.PI/2; // [rad] lower bound of slip angle
      parameter Real z_min = 0.0;                 // [-] lower bound of non-dimensional tire force parameter
      parameter Real z_max = 10^4;                // [-] upper bound of non-dimensional tire force parameter
      parameter Real Vs = 11.1;                 // [V] supply voltage
      parameter Real Np = 4;                    // [-] number of pole pairs in the motor
      parameter Real Nw = 20;                   // [-] number of windings in each phase
      parameter Real A = 0.005*0.02;            // [m^2] cross-sectional area of windings stator
      parameter Real B = 1.2;                   // [T] magnetic flux density of stator
      parameter Real eta_mech = 0.99;           // [-] mechanical conversion efficiency
      parameter Real Kt_phi = eta_mech*Np*Nw*A*B; // [N*m/A] motor torque constant (per-phase)
      parameter Real Kb_phi = Np*Nw*A*B;        // [V*s/rad] motor back emf constant (per-phase)
      parameter Real Kt_q = sqrt(3/2)*Kt_phi;   // [N*m/A] motor torque constant (total)
      parameter Real Kb_q = sqrt(3/2)*Kb_phi;   // [V*s/rad] motor back emf constant (total)
      parameter Real R_phi = 0.1;               // [Ohm] resistance of BLDC motor (per-phase)
      parameter Real J = 1e-4;                  // [kg*m^2] rotor inertia
      parameter Real Le = 1e-2;                 // [H] effective inductance
      parameter Real b = 6.0e-04;               // [N*m*s] viscous friction coefficient
      parameter Real gratio = 2.5;              // [-] gear ratio between motor shaft and wheel shafts
      parameter Real tau_servo = 0.25;          // [sec] time constant of servo
      parameter Real latitude = 40.42362443221589;    // [deg] latitude of the vehicle position in decimal degree
      parameter Real longitude = -86.92702983414662;                          // [deg] longitude of the vehicle position in decimal degree
       
      // inputs
      discrete Real D(start=0,fixed=false);           // [-] PWM duty cycle (0-1), controlled input
      discrete Real delta_cmd(start=0,fixed=false);   // [rad] steering input command
      
      // states & outputs for chassis, tire and motor dynamics
      Real x(start=0,fixed=false), y(start=0,fixed=false), z(start=0,fixed=false);            // [m] position in inertial coordinates
      Real vx(start=0,fixed=false), vy(start=0,fixed=false), vz(start=0,fixed=false);         // [m/s] velocity in body coordinates
      Real ax(start=0,fixed=false), ay(start=0,fixed=false), az(start=0,fixed=false);         // [m/s^2] acceleration in body coordinates
      Real phi(start=0,fixed=false), theta(start=0,fixed=false), psi(start=0,fixed=false);    // [rad] rover attitude (inertial to body)
      Real p(start=0,fixed=false), q(start=0,fixed=false), r(start=0,fixed=false);            // [rad/s] angular velocity
      Real omega_fl(start=0,fixed=false), omega_fr(start=0,fixed=false), omega_rl(start=0,fixed=false), omega_rr(start=0,fixed=false); // [rad/s] rotation speed of tires
      Real kappa_fl(start=0,fixed=false), kappa_fr(start=0,fixed=false), kappa_rl(start=0,fixed=false), kappa_rr(start=0,fixed=false); // [-] longitudinal slip ratio
      Real alpha_fl(start=0,fixed=false), alpha_fr(start=0,fixed=false), alpha_rl(start=0,fixed=false), alpha_rr(start=0,fixed=false); // [-] lateral slip ratio (angle)
      Real Vq(start=0,fixed=false);             // [V] motor voltage (d-q transformation)
      Real Iq(start=0,fixed=false);             // [A] motor current (d-q transformation)
      Real omega(start=0,fixed=false);          // [rad/s] motor shaft rotational speed
      Real lambda(start=0,fixed=false);         // [rad] motor shaft rotation angle
      Real Pmech(start=0,fixed=false);          // [W] mechanical power
      Real Ploss(start=0,fixed=false);          // [W] resistive power loss
      Real delta(start=0,fixed=false);          // [rad] steering input
      Real turn_radius(start=1000);             // [m] turning radius
      Integer rollover_detected(start=0);       // [-] rollover detection signal
      Real mx(start=0,fixed=false), my(start=0,fixed=false), mz(start=0,fixed=false); // [T] magnetic field measured from magnetometer
      
      // internal states
      Real thr;                                       // [N*m] throttle value to each wheel shaft
      Real vs_fl, vs_fr, vs_rl, vs_rr;                // [-] friction reduction magnitude
      Real kappa_fl_bnd, kappa_fr_bnd, kappa_rl_bnd, kappa_rr_bnd;
      Real alpha_fl_bnd, alpha_fr_bnd, alpha_rl_bnd, alpha_rr_bnd;
      Real mu_fl, mu_fr, mu_rl, mu_rr;                // [-] maximum friction coefficient
      Real z_fl, z_fr, z_rl, z_rr;                    // [-] tire force parameter
      Real fz_fl, fz_fr, fz_rl, fz_rr;                // [-] tire force parameter
      Real Fz_fl, Fz_fr, Fz_rl, Fz_rr;                // [N] normal force on wheels
      Real Fx_fl, Fx_fr, Fx_rl, Fx_rr;                // [N] longitudinal force on wheels
      Real Fy_fl, Fy_fr, Fy_rl, Fy_rr;                // [N] lateral force on wheel
      Real vx_fl, vy_fl, vx_fr, vy_fr, vx_rl, vy_rl, vx_rr, vy_rr;                        // [m/s] tire velocity induced by vehicle motion
      
      // accelerometer specific force compensation
      Real specific_g[3](start={0.0, 0.0, Constants.g}, each fixed=false);      // [m/s^2] gravity specific force
      Real C_n2b[3,3](start={{1, 0, 0},{0, 1, 0},{0, 0, 1}},each fixed=false);                    // [-] coordinate transform from NED to body
      
      // load emi & magnetometer model
      Magnetometer mag;
      EMIVulnerability emi(A=A, B=B, Nw=Nw);
    
    // updated: improved tire model eliminates the need for initial equations to make rover exceed speed threshold
/*
    initial equation
      // note: if initialize the shaft speed and wheel speeds using vx value (directly using symbol vx), the initialization fails
      vx = 1.0e-2;                                      // [m/s] initial forward velocity
      omega_fl = 1.0e-2/r_tire;                         // [rad/s] initial front left wheel speed
      omega_fr = 1.0e-2/r_tire;                         // [rad/s] initial front right wheel speed
      omega_rl = 1.0e-2/r_tire;                         // [rad/s] initial rear left wheel speed
      omega_rr = 1.0e-2/r_tire;                         // [rad/s] initial rear right wheel speed
      omega = 1.0e-2/r_tire*gratio;                     // [rad/s] initial motor rotor shaft speed
    */
    
    equation
  
      // compute motor dynamics
      Vq = sqrt(3/2)*Vs*clip(pre(D),1/(sqrt(3/2)*Vs)*(R_phi*b/Kt_q+Kb_q)*(0.001/r_tire*gratio),1);
      der(Iq) = (Vq-R_phi*Iq-Kb_q*omega)/Le;
      der(lambda) = omega;
      when lambda > 2*Constants.PI then
        reinit(lambda,lambda-2*Constants.PI);
      end when;
      der(omega) = (Kt_q*Iq-b*omega-4*thr)/J;
      4*omega/gratio-(omega_fl+omega_fr+omega_rl+omega_rr) = 0;

      // compute servo dynamics
      der(delta) = 1/tau_servo*(pre(delta_cmd)-delta);

      // compute tire force using dugoff tire model with lagged slip ratio
      der(kappa_fl) = -abs(vx+tw/2*r)/clip(tanh(abs(vx)),0.001,1.0)/Lrelx*kappa_fl+(r_tire*omega_fl-(vx+tw/2*r))/Lrelx;//(1+exp(scale*(abs(kappa_fl)-1.00)));
      der(kappa_fr) = -abs(vx-tw/2*r)/clip(tanh(abs(vx)),0.001,1.0)/Lrelx*kappa_fr+(r_tire*omega_fr-(vx-tw/2*r))/Lrelx;//(1+exp(scale*(abs(kappa_fr)-1.00)));
      der(kappa_rl) = -abs(vx+tw/2*r)/clip(tanh(abs(vx)),0.001,1.0)/Lrelx*kappa_rl+(r_tire*omega_rl-(vx+tw/2*r))/Lrelx;//(1+exp(scale*(abs(kappa_rl)-1.00)));
      der(kappa_rr) = -abs(vx-tw/2*r)/clip(tanh(abs(vx)),0.001,1.0)/Lrelx*kappa_rr+(r_tire*omega_rr-(vx-tw/2*r))/Lrelx;      //(1+exp(scale*(abs(kappa_rr)-1.00)));
      kappa_fl_bnd  = clip(kappa_fl,kappa_min,kappa_max);
      kappa_fr_bnd  = clip(kappa_fr,kappa_min,kappa_max);
      kappa_rl_bnd  = clip(kappa_rl,kappa_min,kappa_max);
      kappa_rr_bnd  = clip(kappa_rr,kappa_min,kappa_max);
      
      vx_fl       =   cos(delta)*(vx+tw/2*r)+sin(delta)*(vy+l_front*r);
      vy_fl       =   -sin(delta)*(vx+tw/2*r)+cos(delta)*(vy+l_front*r);
      vx_fr       =   cos(delta)*(vx-tw/2*r)+sin(delta)*(vy+l_front*r);
      vy_fr       =   -sin(delta)*(vx-tw/2*r)+cos(delta)*(vy+l_front*r);
      vx_rl       =   (vx+tw/2*r);
      vy_rl       =   (vy-l_rear*r);
      vx_rr       =   (vx-tw/2*r);
      vy_rr       =   (vy-l_rear*r);
      
      der(alpha_fl) = -abs(vx_fl)*tan(alpha_fl_bnd)/clip(tanh(abs(vx_fl)),0.001,1.0)/Lrely-tanh(abs(vy_fl))*vy_fl/Lrely;
      der(alpha_fr) = -abs(vx_fr)*tan(alpha_fr_bnd)/clip(tanh(abs(vx_fr)),0.001,1.0)/Lrely-tanh(abs(vy_fr))*vy_fr/Lrely;
      der(alpha_rl) = -abs(vx_rl)*tan(alpha_rl_bnd)/clip(tanh(abs(vx_rl)),0.001,1.0)/Lrely-tanh(abs(vy_rl))*vy_rl/Lrely;
      der(alpha_rr) = -abs(vx_rr)*tan(alpha_rr_bnd)/clip(tanh(abs(vx_rr)),0.001,1.0)/Lrely-tanh(abs(vy_rr))*vy_rr/Lrely;
      
      alpha_fl_bnd  = clip(alpha_fl,alpha_min,alpha_max);
      alpha_fr_bnd  = clip(alpha_fr,alpha_min,alpha_max);
      alpha_rl_bnd  = clip(alpha_rl,alpha_min,alpha_max);
      alpha_rr_bnd  = clip(alpha_rr,alpha_min,alpha_max);
      
      vs_fl = (vx+tw/2*r)*sqrt(kappa_fl_bnd^2+tan(alpha_fl_bnd)^2);
      vs_fr = (vx-tw/2*r)*sqrt(kappa_fr_bnd^2+tan(alpha_fr_bnd)^2);
      vs_rl = (vx+tw/2*r)*sqrt(kappa_rl_bnd^2+tan(alpha_rl_bnd)^2);
      vs_rr = (vx-tw/2*r)*sqrt(kappa_rr_bnd^2+tan(alpha_rr_bnd)^2);
      
      mu_fl = clip(mu0*(1-As*vs_fl),0,mu0);
      mu_fr = clip(mu0*(1-As*vs_fr),0,mu0);
      mu_rl = clip(mu0*(1-As*vs_rl),0,mu0);
      mu_rr = clip(mu0*(1-As*vs_rr),0,mu0);
      
      Fz_fl = mass_total*Constants.g*l_rear/(2*l_total)-0.5*(mass_unsprung_front*huf+mass_sprung*hs+mass_unsprung_rear*hur)/l_total*ax+ay/tw*(mass_sprung*l_rear*hs/l_total+mass_unsprung_front*huf)+1/2/tw*(-k_rllsp*phi-c_rllsp*p);
      Fz_fr = mass_total*Constants.g*l_rear/(2*l_total)-0.5*(mass_unsprung_front*huf+mass_sprung*hs+mass_unsprung_rear*hur)/l_total*ax-ay/tw*(mass_sprung*l_rear*hs/l_total+mass_unsprung_front*huf)-1/2/tw*(-k_rllsp*phi-c_rllsp*p);
      Fz_rl = mass_total*Constants.g*l_front/(2*l_total)+0.5*(mass_unsprung_front*huf+mass_sprung*hs+mass_unsprung_rear*hur)/l_total*ax+ay/tw*(mass_sprung*l_front*hs/l_total+mass_unsprung_rear*hur)+1/2/tw*(-k_rllsp*phi-c_rllsp*p);
      Fz_rr = mass_total*Constants.g*l_front/(2*l_total)+0.5*(mass_unsprung_front*huf+mass_sprung*hs+mass_unsprung_rear*hur)/l_total*ax-ay/tw*(mass_sprung*l_front*hs/l_total+mass_unsprung_rear*hur)-1/2/tw*(-k_rllsp*phi-c_rllsp*p);
      
      z_fl = clip(mu_fl*Fz_fl*(1-kappa_fl_bnd)/(2*sqrt((c_kappa*kappa_fl_bnd)^2+(c_alpha*tan(alpha_fl_bnd))^2+Constants.eps)),z_min,z_max);  // revised
      z_fr = clip(mu_fr*Fz_fr*(1-kappa_fr_bnd)/(2*sqrt((c_kappa*kappa_fr_bnd)^2+(c_alpha*tan(alpha_fr_bnd))^2+Constants.eps)),z_min,z_max);
      z_rl = clip(mu_rl*Fz_rl*(1-kappa_rl_bnd)/(2*sqrt((c_kappa*kappa_rl_bnd)^2+(c_alpha*tan(alpha_rl_bnd))^2+Constants.eps)),z_min,z_max);
      z_rr = clip(mu_rr*Fz_rr*(1-kappa_rr_bnd)/(2*sqrt((c_kappa*kappa_rr_bnd)^2+(c_alpha*tan(alpha_rr_bnd))^2+Constants.eps)),z_min,z_max);
      
      if z_fl < 1 then
        fz_fl = z_fl*(2-z_fl);
      else
        fz_fl = 1;
      end if;
      
      if z_fr < 1 then
        fz_fr = z_fr*(2-z_fr);
      else
        fz_fr = 1;
      end if;
      
      if z_rl < 1 then
        fz_rl = z_rl*(2-z_rl);
      else
        fz_rl = 1;
      end if;
      
      if z_rr < 1 then
        fz_rr = z_rr*(2-z_rr);
      else
        fz_rr = 1;
      end if;
      
      Fx_fl = c_kappa*kappa_fl_bnd/(1-kappa_fl_bnd)*fz_fl;
      Fx_fr = c_kappa*kappa_fr_bnd/(1-kappa_fr_bnd)*fz_fr;
      Fx_rl = c_kappa*kappa_rl_bnd/(1-kappa_rl_bnd)*fz_rl;
      Fx_rr = c_kappa*kappa_rr_bnd/(1-kappa_rr_bnd)*fz_rr;
    
      Fy_fl = c_alpha*tan(alpha_fl_bnd)/(1-kappa_fl_bnd)*fz_fl;
      Fy_fr = c_alpha*tan(alpha_fr_bnd)/(1-kappa_fr_bnd)*fz_fr;
      Fy_rl = c_alpha*tan(alpha_rl_bnd)/(1-kappa_rl_bnd)*fz_rl;
      Fy_rr = c_alpha*tan(alpha_rr_bnd)/(1-kappa_rr_bnd)*fz_rr;
  
      // kinematics (planar)
      der(omega_fl) = 1/I_wheel*(thr-r_tire*Fx_fl);
      der(omega_fr) = 1/I_wheel*(thr-r_tire*Fx_fr);
      der(omega_rl) = 1/I_wheel*(thr-r_tire*Fx_rl);
      der(omega_rr) = 1/I_wheel*(thr-r_tire*Fx_rr);
    
      der(x) = vx*cos(psi)-vy*sin(psi);
      der(y) = vx*sin(psi)+vy*cos(psi);
      der(z) = 0;
      der(vx) = vy*r+ax;
      der(vy) = -vx*r-(l_front*mass_unsprung_front-l_rear*mass_unsprung_rear)/mass_total*der(r)+ay;
      der(vz) = 0;
      ax = 1/mass_total*((Fx_fl*cos(-delta)+Fy_fl*sin(-delta))+(Fx_fr*cos(-delta)+Fy_fr*sin(-delta))+(Fx_rl*cos(0)+Fy_rl*sin(0))+(Fx_rr*cos(0)+Fy_rr*sin(0)));
      ay = 1/mass_total*((-Fx_fl*sin(-delta)+Fy_fl*cos(-delta))+(-Fx_fr*sin(-delta)+Fy_fr*cos(-delta))+(-Fx_rl*sin(0)+Fy_rl*cos(0))+(-Fx_rr*sin(0)+Fy_rr*cos(0)));
      az = 0;
      der(phi) = p;
      der(theta) = 0;
      der(psi) = r;
      der(p) = 1/I_xx*(-mass_sprung*ay*hs*cos(phi)+mass_sprung*Constants.g*hs*sin(phi)-k_rllsp*phi-c_rllsp*p);
      der(q) = 0;
      der(r) = 1/I_zz*(tw/2*(Fx_fl*cos(-delta)+Fy_fl*sin(-delta))-tw/2*(Fx_fr*cos(-delta)+Fy_fr*sin(-delta))+tw/2*(Fx_rl*cos(0)+Fy_rl*sin(0))-tw/2*(Fx_rr*cos(0)+Fy_rr*sin(0))+l_front*(-Fx_fl*sin(-delta)+Fy_fl*cos(-delta))+l_front*(-Fx_fr*sin(-delta)+Fy_fr*cos(-delta))-l_rear*(-Fx_rl*sin(0)+Fy_rl*cos(0))-l_rear*(-Fx_rr*sin(0)+Fy_rr*cos(0)));
  
      // compute mechanical power
      Pmech = Kt_q*Iq*omega;
      Ploss = Iq^2*R_phi;
  
      // accelerometer specific force
      C_n2b = transpose(eul2rot({phi, theta, mod(psi + Constants.PI, 2*Constants.PI) - Constants.PI}));
      for i in 1:3 loop
        specific_g[i] = C_n2b[i,3]*Constants.g;
      end for;
  
      // emi model inputs
      emi.Iq = Iq;
      emi.lambda = lambda;
      mag.phi = phi;
      mag.theta = theta;
      mag.psi = psi;
      mx = mag.b_earth[1]+emi.b_wire[1];
      my = mag.b_earth[2]+emi.b_wire[2];
      mz = mag.b_earth[3]+emi.b_wire[3]; 
      
  // rollover check
      when delta <> 0 then
        turn_radius = clip(abs(l_total/tan(delta)),0,1000);
      elsewhen delta == 0 then
        turn_radius = 1000;
      end when;
      
      when Fz_fl <= 1.5 or Fz_fr <= 1.5 or Fz_rl <= 1.5 or Fz_rr <= 1.5 then
        rollover_detected = 1;
      end when;
      
    end RoverHighFidelity;

    model EMIVulnerability                                                 // high-fidelity emi model
      // load packages
      import RoverExample.Utils.cross3;
      import RoverExample.Utils.dot3;
      import RoverExample.Utils.eul2rot;
      import RoverExample.Constants;
      import Modelica.Math.sin;
      import Modelica.Math.cos;
      // parameters for magnetic field model
      parameter Real A;                             // [m^2] cross-sectional area of windings stator
      parameter Real B;                             // [T] magnetic flux density of stator
      parameter Real Nw;                            // [-] number of windings in each phase
      constant Real nu0 = 4*Constants.PI*1e-7;      // [T*m/A] vacuum permeability
      parameter Real mag_motor = B/nu0*(A*0.005);   // [A*m^2] magentic moment of permanent magent on rotor (interior-rotor, per-pole)
      parameter Real eta_motor = 0.05;              // [-] effectiveness of magnetic shielding (0: perfect shielding)
      parameter Real x_motor[3] = {0.03, 0.03, -0.05};  // [m] relative position from motor to magnetometer
      parameter Real dist_motor = sqrt(x_motor[1]^2+x_motor[2]^2+x_motor[3]^2); // [m] given distance from the motor
      parameter Real wire_dir[3] = {-1, 0, 0};      // [-] current flowing wire direction
      parameter Real x_wire[3] = {0.0, 0.0, -0.01}; // [m] relative position from current-flowing wire to magnetometer
      parameter Real dist_wire = sqrt(x_wire[1]^2+x_wire[2]^2+x_wire[3]^2);     // [m] given distance from wire
      // input: current flowing on motor rotational speed of stator
      input Real Iq;                      // [A] current flowing on power line to motor
      input Real lambda;                  // [rad] motor stator position
      // states & outputs for motor, cable harnesses magnetic field
      Real b_motor[3](start={0, 0, 0},each fixed=false); // [T] magnetic flux density
      Real b_wire[3](start={0, 0, 0},each fixed=false);  // [T] magnetic flux density
equation
// compute magnetic field intensity
      b_motor = eta_motor*nu0/(4*Constants.PI)*(-(mag_motor+Nw*Iq*A)*{0,cos(lambda),sin(lambda)}/dist_motor^3+3*dot3((mag_motor+Nw*Iq*A)*{0,cos(lambda),sin(lambda)},x_motor)*x_motor/dist_motor^5);
      b_wire = nu0*Iq/(2*Constants.PI)/dist_wire^2*cross3(wire_dir,x_wire);
    
    annotation(
        Icon(graphics = {Polygon(origin = {47, 8}, lineThickness = 1, points = {{-21, 20}, {-21, -12}, {-19, -16}, {-13, -20}, {-1, -20}, {13, -20}, {19, -16}, {21, -12}, {21, 20}, {11, 20}, {11, -10}, {9, -12}, {-9, -12}, {-11, -10}, {-11, 20}, {-21, 20}, {-21, 20}}), Rectangle(origin = {31, 24}, fillPattern = FillPattern.Solid, extent = {{-5, 4}, {5, -4}}), Rectangle(origin = {63, 24}, fillPattern = FillPattern.Solid, extent = {{-5, 4}, {5, -4}}), Text(origin = {46, -17}, extent = {{-36, 13}, {36, -13}}, textString = "3-axis Magnetometer"), Rectangle(origin = {-49, 1}, fillPattern = FillPattern.Solid, extent = {{-3, 61}, {3, -61}}), Ellipse(origin = {-48, 0}, lineThickness = 1, extent = {{-28, 6}, {28, -6}}), Ellipse(origin = {-48, 0}, lineThickness = 1, extent = {{-38, 12}, {38, -12}}), Ellipse(origin = {-48, 0}, lineThickness = 1, extent = {{-44, 18}, {44, -18}}), Polygon(origin = {-33, 46}, lineThickness = 1, points = {{-1, 10}, {-5, 6}, {-1, 6}, {-1, -10}, {1, -10}, {1, 6}, {5, 6}, {1, 10}, {1, 10}, {-1, 10}}), Text(origin = {-18, 45}, extent = {{-10, 7}, {10, -7}}, textString = "Current"), Polygon(origin = {-94, 2}, fillPattern = FillPattern.Solid, points = {{2, -2}, {-2, 0}, {2, -4}, {6, 0}, {2, -2}}), Polygon(origin = {-66, 4}, fillPattern = FillPattern.Solid, points = {{2, 4}, {-2, 0}, {2, -2}, {0, 0}, {2, 4}}), Polygon(origin = {-86, -4}, fillPattern = FillPattern.Solid, points = {{2, 0}, {-2, 0}, {4, -2}, {6, 2}, {2, 0}})}));
end EMIVulnerability;

    model Magnetometer
      // magnetometer model
      // load packages
      import RoverExample.Utils.cross3;
      import RoverExample.Utils.dot3;
      import RoverExample.Utils.eul2rot;
      import RoverExample.Constants;
      import Modelica.Math.sin;
      import Modelica.Math.cos;
      
      // parameters for magnetic field model
      parameter Real b_earth0 = 3.12e-5;                      // [T] mean magnetic field strength at the magnetic equator on the Earth's surface
      parameter Real lat0 = 40.42244465750271*Constants.d2r;  // [rad] latitude of the rover
      
      // input: current flowing on motor rotational speed of stator
      input Real phi, theta, psi;                             // [rad] vehicle attitude
      
      // states & outputs for motor, cable harnesses magnetic field
      Real b_earth[3](start={0, 0, 0},each fixed=false);                        // [T] magnetic flux density
      Real C_n2b[3,3](start={{1, 0, 0},{0, 1, 0},{0, 0, 1}},each fixed=false);  // [-] coordinate transform from NED to body

    equation
      // compute magnetic field intensity
      C_n2b = transpose(eul2rot({phi, theta, psi}));
      for i in 1:3 loop
        b_earth[i] = C_n2b[i,1]*(b_earth0*sin(Constants.PI/2-lat0))+C_n2b[i,3]*(-2*b_earth0*cos(Constants.PI/2-lat0));
      end for;

    annotation(
        Icon(graphics = {Polygon(origin = {-13, 14}, lineThickness = 1, points = {{-27, 20}, {-27, -24}, {-27, -34}, {-19, -46}, {-7, -54}, {33, -54}, {45, -46}, {53, -34}, {53, 20}, {37, 20}, {37, -30}, {27, -40}, {1, -40}, {-11, -30}, {-11, 20}, {-21, 20}, {-27, 20}}), Rectangle(origin = {-32, 27}, fillPattern = FillPattern.Solid, extent = {{-8, 7}, {8, -7}}), Rectangle(origin = {32, 27}, fillPattern = FillPattern.Solid, extent = {{-8, 7}, {8, -7}}), Text(origin = {2, -51}, extent = {{-36, 13}, {36, -13}}, textString = "3-axis Magnetometer")}));
end Magnetometer;

function MadgwickFusionStep

  // load packages
  import RoverExample.Utils.quatprod;
  import RoverExample.Utils.quatinv;
  import RoverExample.Constants;

  // input: sensor measurements
  input Real ax, ay, az;      // [m/s^2] acceleration measured from accelerometer
  input Real gx, gy, gz;    // [rad/s] rate measured from gyroscope
  input Real mx, my, mz;    // [T] magnetic field measured from magnetometer
      // input: attitude estimate
  input Real q0[4];         
// [-] attitude quaternion
      // parameters for filter
  input Real alpha;         // [-] relative weight for magnetometer
  input Real beta;          // [-] weight for complementary filter
  input Real dt;            
// [sec] sampling time
      // output: updated quaternion
  output Real qnew[4];      
// [-] updated attitude quaternion
    protected
  // internal states
  Real qdot[4];            // [-] quaternion rate
  Real acc_normalized[3];  // [-] normalized acceleration
  Real mag_normalized[3];  // [-] normalized magnetometer
  Real hE_quat[4], bE_quat[4];
  Real fb[3], Jb[4,3], fg[3], Jg[4,3], delta_f[4];

public
  algorithm
// compute quaternion rate
  qdot := 0.5*quatprod(q0,{0, gx, gy, gz});
// normalize acceleration and magnetic field strength
  acc_normalized := {ax, ay, az}/(sqrt(ax^2+ay^2+az^2)+Constants.eps);
  mag_normalized := {mx, my, mz}/(sqrt(mx^2+my^2+mz^2)+Constants.eps);
// earth frame magnetic field computation using previous quaternion
  hE_quat := quatprod(quatprod(q0,{0, mag_normalized[1], mag_normalized[2], mag_normalized[3]}),quatinv(q0));
  bE_quat := {0.0, sqrt(hE_quat[2]^2+hE_quat[3]^2), 0.0, hE_quat[4]};
    
  fb := {2*bE_quat[2]*(1/2-q0[3]^2-q0[4]^2)+2*bE_quat[4]*(q0[2]*q0[4]-q0[1]*q0[3])-mag_normalized[1], 2*bE_quat[2]*(q0[2]*q0[3]-q0[1]*q0[4])+2*bE_quat[4]*(q0[1]*q0[2]+q0[3]*q0[4])-mag_normalized[2], 2*bE_quat[2]*(q0[1]*q0[3]+q0[2]*q0[4])+2*bE_quat[4]*(1/2-q0[2]^2-q0[3]^2)-mag_normalized[3]};

  Jb := {{-2*bE_quat[4]*q0[3], -2*bE_quat[2]*q0[4]+2*bE_quat[4]*q0[2], 2*bE_quat[2]*q0[3]}, {2*bE_quat[4]*q0[4], 2*bE_quat[2]*q0[3]+2*bE_quat[4]*q0[1], 2*bE_quat[2]*q0[4]-4*bE_quat[4]*q0[2]}, {-4*bE_quat[2]*q0[3]-2*bE_quat[4]*q0[1], 2*bE_quat[2]*q0[2]+2*bE_quat[4]*q0[4], 2*bE_quat[2]*q0[1]-4*bE_quat[4]*q0[3]}, {-4*bE_quat[2]*q0[4]+2*bE_quat[4]*q0[2], -2*bE_quat[2]*q0[1]+2*bE_quat[4]*q0[3], 2*bE_quat[2]*q0[2]}};

  fg := {-2*(q0[2]*q0[4]-q0[1]*q0[3])-acc_normalized[1], -2*(q0[1]*q0[2]+q0[3]*q0[4])-acc_normalized[2], -2*(1/2-q0[2]^2-q0[3]^2)-acc_normalized[3]};

  Jg := {{2*q0[3], -2*q0[2], 0.0}, {-2*q0[4], -2*q0[1], 4*q0[2]}, {2*q0[1], -2*q0[4], 4*q0[3]}, {-2*q0[2], -2*q0[3], 0.0}};
  
  delta_f := alpha*Jb*fb+Jg*fg;
  
  qnew := q0+(qdot-beta*delta_f/(sqrt(delta_f[1]^2+delta_f[2]^2+delta_f[3]^2+delta_f[4]^2)+Constants.eps))*dt;
  qnew := qnew/sqrt(qnew[1]^2+qnew[2]^2+qnew[3]^2+qnew[4]^2);

annotation(
        Icon(graphics = {Ellipse(origin = {-63, 10}, lineThickness = 1, extent = {{-7, 20}, {7, -20}}), Polygon(origin = {-56, 8}, fillPattern = FillPattern.Solid, points = {{-2, 2}, {2, 2}, {0, -2}, {-2, 2}, {-2, 2}}), Ellipse(origin = {-64, 11}, lineThickness = 1, extent = {{-20, 7}, {20, -7}}), Polygon(origin = {-70, 12}, fillPattern = FillPattern.Solid, points = {{0, 2}, {-2, -2}, {2, -2}, {0, 2}, {0, 2}}), Polygon(origin = {-61, -56}, lineThickness = 1, points = {{-21, 20}, {-21, -12}, {-19, -16}, {-13, -20}, {-1, -20}, {13, -20}, {19, -16}, {21, -12}, {21, 20}, {11, 20}, {11, -10}, {9, -12}, {-9, -12}, {-11, -10}, {-11, 20}, {-21, 20}, {-21, 20}}), Polygon(origin = {-62, 18}, fillPattern = FillPattern.Solid, points = {{-2, 2}, {-2, -2}, {2, 0}, {2, 0}, {-2, 2}}), Polygon(origin = {-64, 4}, fillPattern = FillPattern.Solid, points = {{2, 2}, {-2, 0}, {2, -2}, {2, 2}, {2, 2}}), Rectangle(origin = {-77, -40}, fillPattern = FillPattern.Solid, extent = {{-5, 4}, {5, -4}}), Rectangle(origin = {-45, -40}, fillPattern = FillPattern.Solid, extent = {{-5, 4}, {5, -4}}), Line(origin = {-62, 70}, points = {{-20, 0}, {20, 0}, {20, 0}}, thickness = 1), Line(origin = {-62, 71}, points = {{0, 19}, {0, -19}, {0, -19}}, thickness = 1), Polygon(origin = {-42, 70}, fillPattern = FillPattern.Solid, points = {{-2, 2}, {-2, -2}, {2, 0}, {2, 0}, {-2, 2}}), Polygon(origin = {-82, 70}, fillPattern = FillPattern.Solid, points = {{2, 2}, {-2, 0}, {2, -2}, {2, 2}, {2, 2}}), Polygon(origin = {-62, 52}, fillPattern = FillPattern.Solid, points = {{-2, 2}, {2, 2}, {0, -2}, {-2, 2}, {-2, 2}}), Polygon(origin = {-62, 90}, fillPattern = FillPattern.Solid, points = {{0, 2}, {-2, -2}, {2, -2}, {0, 2}, {0, 2}}), Polygon(origin = {-46, 62}, fillPattern = FillPattern.Solid, points = {{2, 2}, {-2, -2}, {2, -2}, {2, -2}, {2, 2}}), Polygon(origin = {-78, 78}, fillPattern = FillPattern.Solid, points = {{-2, 2}, {-2, -2}, {2, 2}, {-2, 2}, {-2, 2}}), Line(origin = {-62, 70}, points = {{-16, 8}, {16, -8}, {16, -8}}, thickness = 1), Rectangle(origin = {58, 10}, extent = {{-20, 20}, {20, -20}}), Text(origin = {-62, 43}, extent = {{-36, 13}, {36, -13}}, textString = "3-axis Accelerometer"), Text(origin = {-62, -15}, extent = {{-36, 13}, {36, -13}}, textString = "3-axis Gyroscope"), Text(origin = {-62, -81}, extent = {{-36, 13}, {36, -13}}, textString = "3-axis Magnetometer"), Rectangle(origin = {44, 34}, fillPattern = FillPattern.Solid, extent = {{-2, 4}, {2, -4}}), Ellipse(origin = {51, 21}, extent = {{-3, 3}, {3, -3}}), Ellipse(origin = {49, -1}, extent = {{-3, 3}, {3, -3}}), Ellipse(origin = {71, 7}, extent = {{-3, 3}, {3, -3}}), Ellipse(origin = {58, 10}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Rectangle(origin = {56, 34}, fillPattern = FillPattern.Solid, extent = {{-2, 4}, {2, -4}}), Rectangle(origin = {68, 34}, fillPattern = FillPattern.Solid, extent = {{-2, 4}, {2, -4}}), Rectangle(origin = {72, -14}, fillPattern = FillPattern.Solid, extent = {{-2, 4}, {2, -4}}), Rectangle(origin = {60, -14}, fillPattern = FillPattern.Solid, extent = {{-2, 4}, {2, -4}}), Rectangle(origin = {48, -14}, fillPattern = FillPattern.Solid, extent = {{-2, 4}, {2, -4}}), Line(origin = {55, 14}, points = {{-3, 4}, {3, -4}, {3, -4}}), Line(origin = {53.7929, 4.79289}, points = {{-3.79289, -2.79289}, {2.20711, 3.20711}, {4.20711, 3.20711}}), Line(origin = {63, 9}, points = {{5, -1}, {-5, 1}, {-5, 1}}), Rectangle(origin = {34, 22}, fillPattern = FillPattern.Solid, extent = {{-4, 2}, {4, -2}}), Text(origin = {60, -25}, extent = {{-36, 13}, {36, -13}}, textString = "AHRS (Sensor Fusion)"), Rectangle(origin = {34, 8}, fillPattern = FillPattern.Solid, extent = {{-4, 2}, {4, -2}}), Rectangle(origin = {34, -6}, fillPattern = FillPattern.Solid, extent = {{-4, 2}, {4, -2}}), Rectangle(origin = {82, 26}, fillPattern = FillPattern.Solid, extent = {{-4, 2}, {4, -2}}), Rectangle(origin = {82, 12}, fillPattern = FillPattern.Solid, extent = {{-4, 2}, {4, -2}}), Rectangle(origin = {82, -2}, fillPattern = FillPattern.Solid, extent = {{-4, 2}, {4, -2}}), Line(origin = {-13, 50}, points = {{-13, 10}, {19, -16}, {19, -16}}, thickness = 2), Line(origin = {-12, 10}, points = {{-16, 0}, {16, 0}, {16, 0}}, thickness = 2), Line(origin = {-10, -35}, points = {{-14, -17}, {14, 17}, {14, 17}}, thickness = 2), Polygon(origin = {3, 37}, fillPattern = FillPattern.Solid, points = {{3, 5}, {-5, -5}, {5, -5}, {3, 5}, {3, 5}}), Polygon(origin = {4, 10}, fillPattern = FillPattern.Solid, points = {{-4, 6}, {-4, -6}, {4, 0}, {-4, 6}, {-4, 6}}), Polygon(origin = {3, -19}, fillPattern = FillPattern.Solid, points = {{-5, 3}, {3, -5}, {5, 5}, {-5, 3}, {-5, 3}})}));
end MadgwickFusionStep;

    model GyroAcousticAtk
    // load packages
      import RoverExample.Utils.clip;
      import RoverExample.Utils.avoidzero;
      import RoverExample.Utils.eul2rot;
      import Modelica.Math.sin;
      import Modelica.Math.cos;
      import Modelica.Math.atan;
      import Modelica.Math.log10;
    // system parameter
      parameter Real sample_interval = 0.1;                                     // [sec] update rate for attack
      // targeted acoustic attack model and parameters
      parameter Real m_d = 2.5e-9;                              // [kg] gyroscope driving mass
      parameter Real m_s = 4.1e-9;                              // [kg] gyroscope sensing mass
      parameter Real w_d = 15e+3*2*Constants.PI;                // [rad/s] gyroscope driving frequency
      parameter Real w_s = 23e+3*2*Constants.PI;                // [rad/s] gyroscope sensing natural frequency
      parameter Real k_d = m_d*w_d^2;                           // [N/m] gyroscope driving spring coefficient
      parameter Real k_s = m_s*w_s^2;                           // [N/m] gyroscope sensing spring coefficient
      parameter Real zeta_d = 1/90;                             // [N/m] gyroscope driving damping coefficient (Q-factor: Qd = 45)
      parameter Real zeta_s = 1/36;                             // [N/m] gyroscope sensing damping coefficient (Q-factor: Qs = 18)
      parameter Real dis_d = 7e-6;                              // [m] driving mass displacement
      parameter Real F_d = k_d*dis_d/(1/2/zeta_d);              // [N] driving force needed, small because of very low damping
      parameter Real W = 0;                                     // [W] power of speaker
      parameter Real dist = 0.01;                               // [m] distance to speaker
      parameter Real p0 = 20*10^(-6);                           // [pa] reference pressure
      parameter Real SPL = 10*log10(avoidzero(W/4/Constants.PI/dist^2/(1.21)/(343)/p0^2)); // [pa] sound pressure level
      parameter Real A = p0*10^(SPL/20)*(2*dis_d)^2;            // [N] acoustic force acting on gyro, suppose area = (2*driving displacement)*(2*driving displacement)
      parameter Real psi_ac = 80.0*Constants.d2r;               // [rad] speaker direction
      parameter Real A_x = A*cos(psi_ac);                       // [N] acoustic force on sensing axis, reference - suggested value 4.0e-9
      parameter Real A_y = A*sin(psi_ac);                       // [N] acoustic force on driving axis, reference - suggested value 16.0e-9
      parameter Real w_ac = 15.0002e+3*2*Constants.PI;          // [rad/s] acoustic attack frequency
      parameter Real epsilon = 0.0*Constants.d2r;               // [rad] misalignment of gyroscope, reference - 1deg
      parameter Real phi_0 = 30*Constants.d2r;                  // [rad] phase shift for acoustic noise compared to driving signal
      parameter Real l_g = 1.0e-6;                              // [m] unit length scale
      parameter Real k_bar = k_d/k_s;
      parameter Real w_1 = w_ac/w_d;
      parameter Real w_2 = w_s/w_d;
      parameter Real w_3 = w_ac/w_s;
      parameter Real D_s = F_d/(m_s*w_d^2*l_g);
      parameter Real D_d = F_d/(m_d*w_d^2*l_g);
      parameter Real X_acx = (A_x/(m_s*w_d^2*l_g))/(w_2^2*sqrt((avoidzero(1-w_3^2))^2+(2*zeta_s*w_3)^2));
      parameter Real X_acy = k_bar*sin(epsilon)*(A_y/(m_d*w_d^2*l_g))/sqrt((avoidzero(1-w_3^2))^2+(2*zeta_s*w_3)^2)*sqrt(1+(2*zeta_d*w_1)^2)/sqrt((avoidzero(cos(epsilon)-w_1^2))^2+(2*zeta_d*w_1*cos(epsilon))^2);
      parameter Real X_d1 = sin(epsilon)*D_s/sqrt((avoidzero(1-w_2^2))^2+(2*zeta_s*w_2)^2);
      parameter Real X_d2 = sin(epsilon)*cos(epsilon)*D_s/sqrt((avoidzero(1-w_2^2))^2+(2*zeta_s*w_2)^2)*sqrt(1+(2*zeta_d)^2)/sqrt((avoidzero(cos(epsilon)-1))^2+(2*zeta_d*cos(epsilon))^2);
      parameter Real phi_ac = atan((2*zeta_s*w_3)/(avoidzero(1-w_3^2)));
      parameter Real phi_y = atan((2*zeta_d*w_1*cos(epsilon))/(avoidzero(cos(epsilon)-w_1^2)))+atan((2*zeta_s*w_3)/avoidzero(1-w_3^2))-atan(2*zeta_d*w_1); 
      parameter Real theta_d = atan((2*zeta_s*w_2)/avoidzero(w_2^2-1));
      parameter Real phi_d = atan((2*zeta_d*cos(epsilon))/avoidzero(cos(epsilon)-1))+atan((2*zeta_s*w_2)/avoidzero(w_2^2-1))-atan(2*zeta_d);
      
      parameter Real X_ac = sqrt(X_acx^2+X_acy^2-2*X_acx*X_acy*cos(phi_ac-phi_y));
      parameter Real X_d = sqrt(X_d1^2+X_d1^2-2*X_d1*X_d2*cos(phi_d-theta_d));
      parameter Real Phi_ac = atan((X_acx*sin(phi_0+phi_ac)-X_acy*sin(phi_0+phi_y))/avoidzero(X_acx*cos(phi_0+phi_ac)-X_acy*cos(phi_0+phi_y)));
      parameter Real Phi_d = atan((X_d1*sin(theta_d)-X_d2*sin(phi_d))/avoidzero(X_d1*cos(theta_d)-X_d2*cos(phi_d)));
      // output
      Real omega_yaw_false(start=0,fixed=false);
    // internal state for timer, time needs to be incorprated as state due to FMU sim environment
      discrete Integer timer_count(start=0);
    
    algorithm
    // initialize after a loop
      when sample(0, sample_interval) then
        timer_count := timer_count+1;           // [-] increase the count
      end when;
    
    equation
    // gyroscope acoustic noise injection attack
      omega_yaw_false = l_g/(4*(dis_d/2)/w_d)*(X_ac*cos((w_ac-w_d)*(sample_interval*pre(timer_count))-Phi_ac)+X_d*cos(Phi_d));
    
    annotation(
        Icon(graphics = {Ellipse(origin = {49, 10}, lineThickness = 1, extent = {{-7, 20}, {7, -20}}), Polygon(origin = {56, 8}, fillPattern = FillPattern.Solid, points = {{-2, 2}, {2, 2}, {0, -2}, {-2, 2}, {-2, 2}}), Ellipse(origin = {48, 11}, lineThickness = 1, extent = {{-20, 7}, {20, -7}}), Polygon(origin = {42, 12}, fillPattern = FillPattern.Solid, points = {{0, 2}, {-2, -2}, {2, -2}, {0, 2}, {0, 2}}), Polygon(origin = {50, 18}, fillPattern = FillPattern.Solid, points = {{-2, 2}, {-2, -2}, {2, 0}, {2, 0}, {-2, 2}}), Polygon(origin = {48, 4}, fillPattern = FillPattern.Solid, points = {{2, 2}, {-2, 0}, {2, -2}, {2, 2}, {2, 2}}), Text(origin = {51, -15}, extent = {{-27, 11}, {27, -11}}, textString = "3-axis Gyroscope"), Polygon(origin = {-58, 10}, lineThickness = 1, points = {{-2, 10}, {-22, 10}, {-22, -10}, {-2, -10}, {18, -30}, {22, -30}, {22, 30}, {18, 30}, {-2, 10}, {-2, 10}}), Ellipse(origin = {-20, 10}, lineThickness = 1, extent = {{-2, 10}, {2, -10}}), Ellipse(origin = {-10, 10}, lineThickness = 1, extent = {{-2, 20}, {2, -20}}), Ellipse(origin = {0, 10}, lineThickness = 1, extent = {{-2, 30}, {2, -30}}), Text(origin = {-38, -30}, extent = {{-38, 12}, {38, -12}}, textString = "Ultrasound Transducer")}));
end GyroAcousticAtk;

  end Components;

  package Connectors
    connector IntegerInput = input Integer "'input Integer' as connector" annotation(
      defaultComponentName = "u",
      Icon(graphics = {Polygon(lineColor = {255, 127, 0}, fillColor = {255, 127, 0}, fillPattern = FillPattern.Solid, points = {{-100, 100}, {100, 0}, {-100, -100}, {-100, 100}}), Text(origin = {0, -120}, extent = {{-100, 20}, {100, -20}}, textString = "%name")}, coordinateSystem(extent = {{-100, 100}, {100, -140}}, preserveAspectRatio = true, initialScale = 0.2)),
      Diagram(coordinateSystem(preserveAspectRatio = true, initialScale = 0.2, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(points = {{0, 50}, {100, 0}, {0, -50}, {0, 50}}, lineColor = {255, 127, 0}, fillColor = {255, 127, 0}, fillPattern = FillPattern.Solid), Text(extent = {{-10, 85}, {-10, 60}}, textColor = {255, 127, 0}, textString = "%name")}),
      Documentation(info = "<html>
    <p>
    Connector with one input signal of type Integer.
    </p>
    </html>"));
    
    connector IntegerOutput = output Integer "'output Integer' as connector" annotation(
      defaultComponentName = "y",
      Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, 100}, {100, -100}}), graphics = {Polygon(lineColor = {255, 127, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-100, 100}, {100, 0}, {-100, -100}, {-100, 100}}), Text(origin = {0, -120}, extent = {{-100, 20}, {100, -20}}, textString = "%name")}),
      Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(points = {{-100, 50}, {0, 0}, {-100, -50}, {-100, 50}}, lineColor = {255, 127, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{30, 110}, {30, 60}}, textColor = {255, 127, 0}, textString = "%name")}),
      Documentation(info = "<html>
    <p>
    Connector with one output signal of type Integer.
    </p>
    </html>"));
    
    connector RealOutput = output Real "'output Real' as connector" annotation(
      defaultComponentName = "y",
      Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-100, 100}, {100, 0}, {-100, -100}, {-100, 100}}), Text(origin = {0, -121}, extent = {{-98, 19}, {98, -19}}, textString = "%name")}),
      Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100.0, -100.0}, {100.0, 100.0}}), graphics = {Polygon(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-100.0, 50.0}, {0.0, 0.0}, {-100.0, -50.0}}), Text(textColor = {0, 0, 127}, extent = {{30.0, 60.0}, {30.0, 110.0}}, textString = "%name")}),
      Documentation(info = "<html>
    <p>
    Connector with one output signal of type Real.
    </p>
    </html>"));
    
    connector RealInput = input Real "'input Real' as connector" annotation(
      defaultComponentName = "u",
      Icon(graphics = {Polygon(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, points = {{-100, 100}, {100, 0}, {-100, -100}, {-100, 100}}), Text(origin = {-1, -120}, extent = {{-101, 20}, {101, -20}}, textString = "%name")}, coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.2)),
      Diagram(coordinateSystem(preserveAspectRatio = true, initialScale = 0.2, extent = {{-100.0, -100.0}, {100.0, 100.0}}), graphics = {Polygon(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, points = {{0.0, 50.0}, {100.0, 0.0}, {0.0, -50.0}, {0.0, 50.0}}), Text(textColor = {0, 0, 127}, extent = {{-10.0, 60.0}, {-10.0, 85.0}}, textString = "%name")}),
      Documentation(info = "<html>
    <p>
    Connector with one input signal of type Real.
    </p>
    </html>"));

    expandable connector ControlBus
  annotation(
        Icon(graphics = {Polygon(origin = {0, -20}, points = {{-50, -20}, {-80, 40}, {80, 40}, {50, -20}, {-50, -20}}), Ellipse(origin = {17, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-19, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-51, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {49, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-31, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-1, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {29, -24}, extent = {{-9, 8}, {9, -8}}), Text(origin = {0, -60}, extent = {{-80, 20}, {80, -20}}, textString = "%name")}, coordinateSystem(extent = {{-80, 20}, {80, -80}})),
  Diagram(graphics = {Polygon(origin = {0, -20}, points = {{-50, -20}, {-80, 40}, {80, 40}, {50, -20}, {-50, -20}}), Ellipse(origin = {17, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-19, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-51, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {49, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-31, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-1, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {29, -24}, extent = {{-9, 8}, {9, -8}}), Text(origin = {0, -60}, extent = {{-80, 20}, {80, -20}}, textString = "%name")}, coordinateSystem(extent = {{-80, 20}, {80, -80}})));
    end ControlBus;
    
    expandable connector UdpBus
    
  annotation(
        Icon(graphics = {Polygon(origin = {0, -20}, points = {{-50, -20}, {-80, 40}, {80, 40}, {50, -20}, {-50, -20}}), Ellipse(origin = {17, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-19, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-51, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {49, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-31, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-1, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {29, -24}, extent = {{-9, 8}, {9, -8}}), Text(origin = {0, -60}, extent = {{-80, 20}, {80, -20}}, textString = "%name")}, coordinateSystem(extent = {{-80, 20}, {80, -80}})),
  Diagram(graphics = {Polygon(origin = {0, -20}, points = {{-50, -20}, {-80, 40}, {80, 40}, {50, -20}, {-50, -20}}), Ellipse(origin = {17, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-19, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-51, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {49, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-31, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-1, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {29, -24}, extent = {{-9, 8}, {9, -8}}), Text(origin = {0, -60}, extent = {{-80, 20}, {80, -20}}, textString = "%name")}, coordinateSystem(extent = {{-80, 20}, {80, -80}})));
    end UdpBus;
    
    expandable connector PwmBus
  annotation(
        Icon(graphics = {Polygon(origin = {0, -20}, points = {{-50, -20}, {-80, 40}, {80, 40}, {50, -20}, {-50, -20}}), Ellipse(origin = {17, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-19, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-51, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {49, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-31, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-1, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {29, -24}, extent = {{-9, 8}, {9, -8}}), Text(origin = {0, -60}, extent = {{-80, 20}, {80, -20}}, textString = "%name")}, coordinateSystem(extent = {{-80, 20}, {80, -80}})),
  Diagram(graphics = {Polygon(origin = {0, -20}, points = {{-50, -20}, {-80, 40}, {80, 40}, {50, -20}, {-50, -20}}), Ellipse(origin = {17, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-19, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-51, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {49, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-31, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-1, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {29, -24}, extent = {{-9, 8}, {9, -8}}), Text(origin = {0, -60}, extent = {{-80, 20}, {80, -20}}, textString = "%name")}, coordinateSystem(extent = {{-80, 20}, {80, -80}})));
    end PwmBus;
    
    expandable connector SensorBus
  annotation(
        Icon(graphics = {Polygon(origin = {0, -20}, points = {{-50, -20}, {-80, 40}, {80, 40}, {50, -20}, {-50, -20}}), Ellipse(origin = {17, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-19, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-51, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {49, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-31, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-1, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {29, -24}, extent = {{-9, 8}, {9, -8}}), Text(origin = {0, -60}, extent = {{-80, 20}, {80, -20}}, textString = "%name")}, coordinateSystem(extent = {{-80, 20}, {80, -80}})),
  Diagram(graphics = {Polygon(origin = {0, -20}, points = {{-50, -20}, {-80, 40}, {80, 40}, {50, -20}, {-50, -20}}), Ellipse(origin = {17, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-19, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-51, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {49, 0}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-31, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {-1, -24}, extent = {{-9, 8}, {9, -8}}), Ellipse(origin = {29, -24}, extent = {{-9, 8}, {9, -8}}), Text(origin = {0, -60}, extent = {{-80, 20}, {80, -20}}, textString = "%name")}, coordinateSystem(extent = {{-80, 20}, {80, -80}})));
    end SensorBus;
  end Connectors;

  package Constants
      constant Real PI = 3.14159265;
      constant Real g = 9.80665;
      constant Real eps = 1.0e-15;
      constant Real r2d = 180/PI;
      constant Real d2r = PI/180;
  
  end Constants;

  package Utils
    function clip
    
      input Real x;       // [-] input value
      input Real x_min;   // [-] minimum limit
      input Real x_max;   // [-] maximum limit
      output Real y;            // [-] clipped value
    algorithm
    
      y := if x < x_min then x_min else if x > x_max then x_max else x;
      
    end clip;

    function avoidzero
    
      import RoverExample.Constants;  // [-] load constants
      input Real x;                   // [-] input value
      output Real y;                  
    // [-] output value
algorithm
    
      if abs(x) < Constants.eps then
        y := Constants.eps * sign(x + Constants.eps);   // preserve sign, avoid exact zero
      else
        y := x;
      end if;
      
    end avoidzero;

    function quatprod
    
      input Real q_in1[4], q_in2[4];  // [-] input quaternion
      output Real q_out[4];               // [-] output quaternion
    algorithm
    
      q_out[1] := q_in1[1]*q_in2[1]-q_in1[2]*q_in2[2]-q_in1[3]*q_in2[3]-q_in1[4]*q_in2[4];
      q_out[2] := q_in1[1]*q_in2[2]+q_in1[2]*q_in2[1]+q_in1[3]*q_in2[4]-q_in1[4]*q_in2[3];
      q_out[3] := q_in1[1]*q_in2[3]-q_in1[2]*q_in2[4]+q_in1[3]*q_in2[1]+q_in1[4]*q_in2[2];
      q_out[4] := q_in1[1]*q_in2[4]+q_in1[2]*q_in2[3]-q_in1[3]*q_in2[2]+q_in1[4]*q_in2[1];
      
    end quatprod;

    function quat2rot
    
      input Real q[4];      // [-] input quaternion (a point rotation)
      output Real R[3,3];       // [-] output rotation matrix (a point rotation)
    algorithm
    
      R[1,1] := q[1]*q[1]+q[2]*q[2]-q[3]*q[3]-q[4]*q[4];
      R[1,2] := 2*(q[2]*q[3]-q[1]*q[4]);
      R[1,3] := 2*(q[2]*q[4]+q[1]*q[3]);
      R[2,1] := 2*(q[2]*q[3]+q[1]*q[4]);
      R[2,2] := q[1]*q[1]+q[3]*q[3]-q[2]*q[2]-q[4]*q[4];
      R[2,3] := 2*(q[3]*q[4]-q[1]*q[2]);
      R[3,1] := 2*(q[2]*q[4]-q[1]*q[3]);
      R[3,2] := 2*(q[3]*q[4]+q[1]*q[2]);
      R[3,3] := q[1]*q[1]+q[4]*q[4]-q[2]*q[2]-q[3]*q[3];
    
    end quat2rot;

    function quat2eul
    
      import Modelica.Math.atan2; // [-] load atan2 function
      import Modelica.Math.asin;  // [-] load asin function
      input Real q[4];            // [-] input quaternion (a point rotation)
      output Real euler[3];           // [-] output euler angle (a point rotation)
    algorithm
    
      euler[1] := atan2(2*q[3]*q[4]+2*q[1]*q[2],2*q[1]^2+2*q[4]^2-1);
      euler[2] := -asin(2*q[2]*q[4]-2*q[1]*q[3]);
      euler[3] := atan2(2*q[2]*q[3]+2*q[1]*q[4],2*q[1]^2+2*q[2]^2-1);

    end quat2eul;

    function quatinv
    
      input Real q_in[4];   // [-] input quaternion
      output Real q_out[4];     // [-] output quaternion
    algorithm
    
      q_out[1] := q_in[1];
      q_out[2] := -q_in[2];
      q_out[3] := -q_in[3];
      q_out[4] := -q_in[4];

    end quatinv;

    function dot3
    
      input Real v_in1[3], v_in2[3];  // [-] input vector
      output Real v_out;                  // [-] output vector
    algorithm
    
      v_out := v_in1[1]*v_in2[1]+v_in1[2]*v_in2[2]+v_in1[3]*v_in2[3];
      
    end dot3;

    function cross3
    
      input Real v_in1[3], v_in2[3];  // [-] input vector
      output Real v_out[3];               // [-] output vector
    algorithm
    
      v_out[1] := v_in1[2]*v_in2[3]-v_in1[3]*v_in2[2];
      v_out[2] := -(v_in1[1]*v_in2[3]-v_in1[3]*v_in2[1]);
      v_out[3] := v_in1[1]*v_in2[2]-v_in1[2]*v_in2[1];

    end cross3;

    function eul2rot
    
      import Modelica.Math.sin;     // [-] load sine function
      import Modelica.Math.cos;     // [-] load cosine function
      input Real euler[3];          // [rad] euler angle
      output Real R[3,3];               // [-] output rotation matrix (a point rotation)
    algorithm
    
      R[1,1] := cos(euler[2])*cos(euler[3]);
      R[1,2] := -cos(euler[1])*sin(euler[3])+sin(euler[1])*sin(euler[2])*cos(euler[3]);
      R[1,3] := sin(euler[1])*sin(euler[3])+cos(euler[1])*sin(euler[2])*cos(euler[3]);
      R[2,1] := cos(euler[2])*sin(euler[3]);
      R[2,2] := cos(euler[1])*cos(euler[3])+sin(euler[1])*sin(euler[2])*sin(euler[3]);
      R[2,3] := -sin(euler[1])*cos(euler[3])+cos(euler[1])*sin(euler[2])*sin(euler[3]);
      R[3,1] := -sin(euler[2]);
      R[3,2] := sin(euler[1])*cos(euler[2]);
      R[3,3] := cos(euler[1])*cos(euler[2]);

    end eul2rot;

    function eul2quat
    
      import Modelica.Math.sin;     // [-] load sine function
      import Modelica.Math.cos;     // [-] load cosine function
      input Real euler[3];          // [rad] euler angle
      output Real q[4];                   // [-] output quaternion
    algorithm
    
      q[1] := cos(euler[3]/2)*cos(euler[2]/2)*cos(euler[1]/2)+sin(euler[3]/2)*sin(euler[2]/2)*sin(euler[1]/2);
      q[2] := cos(euler[3]/2)*cos(euler[2]/2)*sin(euler[1]/2)-sin(euler[3]/2)*sin(euler[2]/2)*cos(euler[1]/2);
      q[3] := cos(euler[3]/2)*sin(euler[2]/2)*cos(euler[1]/2)+sin(euler[3]/2)*cos(euler[2]/2)*sin(euler[1]/2);
      q[4] := sin(euler[3]/2)*cos(euler[2]/2)*cos(euler[1]/2)-cos(euler[3]/2)*sin(euler[2]/2)*sin(euler[1]/2);

    end eul2quat;
  end Utils;
end RoverExample;
