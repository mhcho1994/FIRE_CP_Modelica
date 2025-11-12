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
  experiment(StartTime = 0, StopTime = 30.0, Tolerance = 1e-06, Interval = 0.02));
  
  end ExampleScenario;
  
// (redeclare model RoverModel = RoverExample.Components.RoverHighFidelity)

  package Components
    model Webserver
    // interaction of the user-making discrete turn commands on the website
      parameter Real sample_interval = 0.1;      // [sec] update rate for command
      parameter Real turn_interval = 15;//7.0;       // [sec] interval between straight, left, right sequence
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
      //if (timer_count <= 1*turn_interval_count) and (timer_count >= 0*turn_interval_count) then
      //  turn := 0;
        
      //elseif (timer_count <= 2*turn_interval_count) and (timer_count > 1*turn_interval_count) then
      //  turn := 1;
        
      //elseif (timer_count <= 3*turn_interval_count) and (timer_count > 2*turn_interval_count) then
      //  turn := 1;
        
      //else
      //  turn := 0;
        
      //end if;
// comment out: single turn simulation

      //if timer_count >= repeat_interval_count then
      //  timer_count := 0;
      //end if;
      
      //turn := 0;
    
      if (timer_count <= 1*turn_interval_count) then
        turn := 0;
      else
        turn := 1;
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
      parameter Real delta_turn = 3.28*Constants.PI/180;       // [rad] turn steering angle
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
        pwm_throttle := 2000; //1000 + integer(1000*v/v_max);
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
      Connectors.RealOutput rollover_metric_roll;
      Connectors.RealOutput rollover_metric_z;
      Connectors.RealOutput shaft_failure;
      
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
        shaft_failure = 0;
        rollover_metric_roll = 0;
        rollover_metric_z = 0;
        
      elseif fidelity == 2 then
        x_meas = rover_8d.x_t;
        y_meas = rover_8d.y_u;
        z_meas = rover_8d.z_s;
        vx_meas = rover_8d.u_t;
        vy_meas = rover_8d.v_u;
        vz_meas = rover_8d.w_s;
        ax_meas = rover_8d.a_x-rover_8d.specific_g[1];
        ay_meas = rover_8d.a_y_s-rover_8d.specific_g[2];
        az_meas = 0-rover_8d.specific_g[3];
        phi_meas = rover_8d.phi_s;
        theta_meas = rover_8d.theta_s;
        psi_meas = mod(rover_8d.psi_t + Constants.PI, 2*Constants.PI) - Constants.PI;
        p_meas = rover_8d.p_s;
        q_meas = rover_8d.q_s;
        r_meas = rover_8d.r_t+gyroatk.omega_yaw_false;
        mx_meas = rover_8d.mx;
        my_meas = rover_8d.my;
        mz_meas = rover_8d.mz;
        rollover_detected = rover_8d.rollover_detected;
        shaft_failure = rover_8d.shaft_failure;
        rollover_metric_roll = rover_8d.rollover_metric_roll; // 28deg
        rollover_metric_z = rover_8d.rollover_metric_z; // -0.056 m
        
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
      ay = thr*thr/l_total*tan(delta);
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
      parameter Real mass_unsprung_front = 6.0*mass_wheel;  // [kg] front unsprung mass
      parameter Real mass_unsprung_rear = 6.0*mass_wheel;   // [kg] rear unsprung mass
      parameter Real mass_sprung = mass_total-mass_unsprung_front-mass_unsprung_rear;
      parameter Real l_total = 0.275;                       // [m] distance from rear axle to front axle
      parameter Real l_front = 0.136;                       // [m] distance from cg to front axle
      parameter Real l_rear = 0.139;                        // [m] distance from cg to rear axle
      parameter Real tw = 0.234;                            // [m] rover trackwidth
      parameter Real r_wheel = 0.056;                       // [m] rover tire radius
      parameter Real w_wheel = 0.06;                        // [m] tire width
      parameter Real h_raf = 0.076;                         // [m] height of front unsprung mass cg from roll axis
      parameter Real h_rar = 0.076;                         // [m] height of rear unsprung mass cg from roll axis
      parameter Real h_s = 0.101;                           // [m] height of rear sprung mass cg from roll axis
      parameter Real Iyy_wheel = 1/2*mass_wheel*r_wheel^2*3;               // [kg*m^2] y-axis moment of inertia of wheels
      parameter Real Izz_wheel = 1/12*mass_wheel*(3*r_wheel^2+w_wheel^2);  // [kg*m^2] z-axis moment of inertia of wheels
      parameter Real Izz_t = 1/12*mass_total*(l_total^2+tw^2);              // [kg*m^2] z-axis moment of inertia of total chassis
      parameter Real Ixx_uf = 1/12*mass_unsprung_front*(tw^2+(2*(h_raf-r_wheel))^2);  // [kg*m^2] x-axis moment of inertia of front unsprung mass
      parameter Real Ixx_ur = 1/12*mass_unsprung_rear*(tw^2+(2*(h_rar-r_wheel))^2);   // [kg*m^2] x-axis moment of inertia of rear unsprung mass
      parameter Real Ixx_s = 1/12*mass_sprung*(tw^2+(2*(h_s-h_raf))^2);               // [kg*m^2] x-axis moment of inertia of sprung mass
      parameter Real Ixz_s = 0;                                                       // [kg*m^2] xz product inertia of sprung mass
      parameter Real Iyy_s = 1/12*mass_sprung*(tw^2+l_total^2);                       // [kg*m^2] y-axis moment of inertia of sprung mass
    
      parameter Real mu0 = 0.9;                 // [-] maximum friction scaling coefficient, [0.4 1.2]
      parameter Real As = 0.0;                  // [-] friction reduction factor
      parameter Real c_s = 400.0;               // [N] longitudinal stiffness
      parameter Real c_alpha = 2400.0;          // [N/rad] lateral stiffness
      parameter Real Lrelx = 0.185;             // [m] longitudinal relaxation length
      parameter Real Lrely = 0.185;             // [m] lateral realaxation length
    
      parameter Real k_lt = 0;                              // [-] lateral compliance rate of tire and suspension, cf. 2.6/k_zt
      parameter Real k_scf = (20.0^2)*(2*Izz_wheel);       // [rad/N*m] steering compliance for front steering gear, wn = 20 rad/s
      parameter Real k_scv = (2*0.707*20.0)*(2*Izz_wheel); // [rad*s/N*m] damping for front steering gear, zeta = 0.707
      parameter Real k_zt = 510*(30/25.4+2*(56/25.4))*(42/35)*0.3048/4.44822; // [N/m] vertical spring rate of tire, cf. REF 5 page B-14, tune pressure from 35psi to 42psi
      parameter Real k_zd = 0.1*k_zt;                       // [N*s/m] vertical damping of tire, added for stability
    
      parameter Real k_sf = (0.5*mass_sprung*l_rear/l_total)*(24.36)^2;               // [N/m] front suspension spring rate (adapted from experimental data)
      parameter Real k_sdf = 2*(0.12)*sqrt(k_sf*(0.5*mass_sprung*l_rear/l_total));    // [N*s/m] front suspension damping rate (adapted from experimental data)
      parameter Real k_orsf = 25.0;                                                   // [N*m/rad] overall roll stiffness of front axle
      parameter Real k_tsf = k_orsf*(k_zt*tw^2/2)/((k_zt*tw^2/2)-k_orsf)+k_sf*tw^2/2; // [N*m/rad] front auxiliary roll stiffness
      parameter Real l_saf = 2/3*tw;                                                  // [m] length of the front axle arm (wheel to suspension pivot)
      parameter Real tw_f = tw;                                                       // [m] front trackwidth
      parameter Real k_slf = 0.115;                                                   // [-] lateral slope of equivalent single suspension arm, cf. REF 5 page C-2
      parameter Real k_sadf = 0.0;                                                    // [-] suspension squat/lift ratio (in deceleration)
      parameter Real k_sad2f = 0.0;                                                   // [-] suspension squat/lift ratio (in acceleration)
      parameter Real k_sr = (0.5*mass_sprung*l_front/l_total)*(24.36)^2;              // [N/m] front suspension spring rate (adapted from experimental data)
      parameter Real k_sdr = 2*(0.12)*sqrt(k_sr*(0.5*mass_sprung*l_front/l_total));   // [N*s/m] front suspension damping rate (adapted from experimental data)
      parameter Real k_orsr = k_orsf;                                                 // [N*m/rad] overall roll stiffness of front axle
      parameter Real k_tsr = k_orsr*(k_zt*tw^2/2)/((k_zt*tw^2/2)-k_orsr)+k_sr*tw^2/2; // [N*m/rad] front auxiliary roll stiffness
      parameter Real l_sar = 2/3*tw;                                                  // [m] length of the front axle arm (wheel to suspension pivot)
      parameter Real tw_r = tw;                                                       // [m] front trackwidth
      parameter Real k_slr = 0.115;                                                   // [-] lateral slope of equivalent single suspension arm, cf. REF 5 page C-2
      parameter Real k_sadr = 0.0;                                                    // [-] suspension squat/lift ratio (in deceleration)
      parameter Real k_sad2r = 0.0;                                                   // [-] suspension squat/lift ratio (in acceleration)
      parameter Real h_bs = 0.03;                                                     // [m] suspension travel to bumpt stop (measured)
      parameter Real k_bs = 2.0*k_sf;                                                 // [N/m] bump stop stiffness, cf. REF 5 page B-13
      parameter Real k_ras = 1.0*10^3;                                                // [N/m] lateral spring rate at compliant pin joint between sprung and unsprung masses, cf. REF 5 page B-13
      parameter Real k_rad = 1.0*2*sqrt(2*mass_unsprung_front*k_ras);                 // [N*s/m] lateral damping rate at compliant pin joint between sprung and unsprung masses, cf. REF 5 page B-14
      parameter Real s_max = 0.999;               // [-] upper bound of slip ratio
      parameter Real s_min = -0.999;              // [-] lower bound of slip ratio
      parameter Real alpha_max = Constants.PI/2;  // [rad] upper bound of slip angle
      parameter Real alpha_min = -Constants.PI/2; // [rad] lower bound of slip angle
      parameter Real kappa_min = 0.0;             // [-] lower bound of non-dimensional tire force parameter
      parameter Real kappa_max = 10^4;            // [-] upper bound of non-dimensional tire force parameter
      
      parameter Real Vs = 11.1;                   // [V] supply voltage
      parameter Real Np = 4;                      // [-] number of pole pairs in the motor
      parameter Real Nw = 20;                     // [-] number of windings in each phase
      parameter Real A = 0.005*0.02;              // [m^2] cross-sectional area of windings stator
      parameter Real B = 1.2;                     // [T] magnetic flux density of stator
      parameter Real eta_mech = 0.99;             // [-] mechanical conversion efficiency
      parameter Real Kt_phi = eta_mech*Np*Nw*A*B; // [N*m/A] motor torque constant (per-phase)
      parameter Real Kb_phi = Np*Nw*A*B;          // [V*s/rad] motor back emf constant (per-phase)
      parameter Real Kt_q = sqrt(3/2)*Kt_phi;     // [N*m/A] motor torque constant (total)
      parameter Real Kb_q = sqrt(3/2)*Kb_phi;     // [V*s/rad] motor back emf constant (total)
      parameter Real R_phi = 0.1;                 // [Ohm] resistance of BLDC motor (per-phase)
      parameter Real Jm = 1e-4;                   // [kg*m^2] rotor inertia
      parameter Real Le = 1e-2;                   // [H] effective inductance
      parameter Real b = 6.0e-04;                 // [N*m*s] viscous friction coefficient
      parameter Real gratio = 2.5;                // [-] gear ratio between motor shaft and wheel shafts
    
      parameter Real delta_max = (28.28-3.00)/180*Constants.PI; // [rad] maximum angle of steering servo
      parameter Real deltadot_max = 100/180*Constants.PI;           // [rad/s] maximum
      parameter Real latitude = 40.42362443221589;    // [deg] latitude of the vehicle position in decimal degree
      parameter Real longitude = -86.92702983414662;                          // [deg] longitude of the vehicle position in decimal degree
       
      parameter Real torqueGain = 0.08;
       
      // parameters for chassis, tire and motor dynamics
      parameter Real tau_yield = 0.577 * 140 * 10^6;		// [pa] yield shear stress
      parameter Real r_shaft = 3e-3;			// [m] radius of shaft
       
      // states & outputs for chassis, tire and motor dynamics
      Integer shaft_failure(start=0);       // [-] shaft failure signal
      Real T_motor(start=0,fixed=false);            // [N*m] motor torque out from motor
      Real T_gear(start=0,fixed=false);            // [N*m] motor torque out from gearbox
      Real tau_shaft(start=0,fixed=false);	// [N/m^2] shear stress applied to shaft
    
       
      // inputs
      discrete Real D(start=0,fixed=false);           // [-] PWM duty cycle (0-1), controlled input
      discrete Real delta_cmd(start=0,fixed=false);   // [rad] steering input command
      
      //parameter Real D = 0.15;
      //parameter Real delta_cmd = 0;
      
      // states & outputs for chassis, tire and motor dynamics
      // global pose & planar chassis velocities
      Real x_t(start=0,fixed=false), u_t(start=0,fixed=false);    // [m, m/s] x position in inertial coordinates, forward velociy in road coordinates
      Real psi_t(start=0,fixed=false), r_t(start=0,fixed=false);  // [rad] heading and yaw rate in inertial coordinates
      Real y_u(start=0,fixed=false), v_u(start=0,fixed=false);    // [m, m/s] y position in inertial coordinates (unsprung masses), side velocity in road coordinates
    
      // lateral relative deflections/velocities
      Real delta_y_suf(start=0,fixed=false), delta_v_suf(start=0,fixed=false);
      Real delta_y_sur(start=0,fixed=false), delta_v_sur(start=0,fixed=false);
    
      // vertical positions/velocities
      Real z_uf(start=0,fixed=false), w_uf(start=0,fixed=false);
      Real z_ur(start=0,fixed=false), w_ur(start=0,fixed=false);
      Real z_s(start=0,fixed=false), w_s(start=0,fixed=false);
    
      // attitudes/rates
      Real phi_uf(start=0,fixed=false), p_uf(start=0,fixed=false);
      Real phi_ur(start=0,fixed=false), p_ur(start=0,fixed=false);
      Real phi_s(start=0,fixed=false), p_s(start=0,fixed=false);
      Real theta_s(start=0,fixed=false), q_s(start=0,fixed=false);
    
      // wheel spins
      Real omega_fl(start=0,fixed=false), omega_fr(start=0,fixed=false), omega_rl(start=0,fixed=false), omega_rr(start=0,fixed=false); // [rad/s] rotation speed of tires
      
      // Motor dq current & shaft speed
      Real Vq(start=0,fixed=false);             // [V] motor voltage (d-q transformation)
      Real Iq(start=0), omega_m(start=0);
      Real lambda_m(start=0,fixed=false);         // [rad] motor shaft rotation angle
    
    
      // Longitudinal slip (4) & lateral slip (4)
      Real s_fl(start=0), s_fr(start=0), s_rl(start=0), s_rr(start=0);                  // [-] longitudinal slip ratio
      Real alpha_fl(start=0), alpha_fr(start=0), alpha_rl(start=0), alpha_rr(start=0);
    
      // Steering state (angle & rate) — compliant steering
      Real delta_f(start=0), deltadot_f(start=0);
    
      // ===== Derived locals =====
      // Ackermann steer per front wheel
      Real delta_fl, delta_fr, delta_rl, delta_rr;
    
      // Tire normal forces
      Real Fz_LF, Fz_RF, Fz_LR, Fz_RR;
    
      // Wheel-plane velocities
      Real u_fl, v_fl, u_fr, v_fr, u_rl, v_rl, u_rr, v_rr;
    
      // Slip helper
      Real mu_fl, mu_fr, mu_rl, mu_rr, kappa_fl, kappa_fr, kappa_rl, kappa_rr, fkappa_fl, fkappa_fr, fkappa_rl, fkappa_rr;
      Real vs_fl, vs_fr, vs_rl, vs_rr;
    
      // Tire forces in tire frame
      Real Fx_LF, Fx_RF, Fx_LR, Fx_RR;
      Real Fy_LF, Fy_RF, Fy_LR, Fy_RR;
    
      // Suspension forces
      Real z_SLF, z_SRF, z_SLR, z_SRR;
      Real zdot_SLF, zdot_SRF, zdot_SLR, zdot_SRR;
      Real F_BSLF, F_BSRF, F_BSLR, F_BSRR;
      Real F_SQLF, F_SQRF, F_SQLR, F_SQRR;
      Real F_SLF,  F_SRF,  F_SLR,  F_SRR;
      Real F_RAF,  F_RAR;
    
      // Chassis accels (planar) and coupled yaw/roll ddots
      Real a_x, a_y_uf, a_y_ur, a_y_s;
      Real rdot, pdot_s, psiddot, phiddot_s;
      
      Real Pmech(start=0,fixed=false);          // [W] mechanical power
      Real Ploss(start=0,fixed=false);          // [W] resistive power loss
      Real turn_radius(start=1000);             // [m] turning radius
      Integer rollover_detected(start=0);       // [-] rollover detection signal
      Real mx(start=0,fixed=false), my(start=0,fixed=false), mz(start=0,fixed=false); // [T] magnetic field measured from magnetometer
      Real rollover_metric_roll;
      Real rollover_metric_z;
      
      // accelerometer specific force compensation
      Real specific_g[3](start={0.0, 0.0, Constants.g}, each fixed=false);      // [m/s^2] gravity specific force
      Real C_n2b[3,3](start={{1, 0, 0},{0, 1, 0},{0, 0, 1}},each fixed=false);                    // [-] coordinate transform from NED to body
      
      // load emi & magnetometer model
      Magnetometer mag;
      EMIVulnerability emi(A=A, B=B, Nw=Nw);
    
    equation
  
      // compute motor dynamics
      Vq = sqrt(3/2)*Vs*clip(pre(D),1/(sqrt(3/2)*Vs)*(R_phi*b/Kt_q+Kb_q)*(0.001/r_wheel*gratio),1);
      
    
      
      
      der(Iq) = (Vq-R_phi*Iq-Kb_q*omega_m)/Le;
      der(lambda_m) = omega_m;
      when lambda_m > 2*Constants.PI then
        reinit(lambda_m,lambda_m-2*Constants.PI);
      end when;
      
      // compute shear stress
      T_motor = Kt_q * Iq;
      T_gear = T_motor *eta_mech * gratio;
      tau_shaft = T_gear * 2 / Constants.PI / (r_shaft^3);
      
      der(omega_m) = (T_motor-b*omega_m-torqueGain*((omega_m/gratio - omega_fl) + (omega_m/gratio - omega_fr) + (omega_m/gratio - omega_rl) + (omega_m/gratio - omega_rr)))/Jm;

      // compute servo dynamics
      der(delta_f)   = deltadot_f;
      der(deltadot_f)= ( (delta_cmd - delta_f) - k_scv*deltadot_f ) / (2*Iyy_wheel*k_scf);
    
      // Ackermann split (front), rear fixed
      delta_fl = atan( tan(delta_f) / (1 + tw/(2*l_total)*tan(delta_f)) );
      delta_fr = atan( tan(delta_f) / (1 - tw/(2*l_total)*tan(delta_f)) );
      delta_rl = 0; delta_rr = 0;
    
      // ===== Tire normal forces (vertical tire spring/damper) =====
      Fz_LF = mass_sprung*Constants.g*l_rear/(2*l_total) + mass_unsprung_front*Constants.g/2 + (z_uf + r_wheel*(cos(phi_uf)-1) - tw/2*sin(phi_uf))*k_zt + (w_uf - r_wheel*(sin(phi_uf)*p_uf) - tw/2*cos(phi_uf)*p_uf)*k_zd;
      Fz_RF = mass_sprung*Constants.g*l_rear/(2*l_total) + mass_unsprung_front*Constants.g/2 + (z_uf + r_wheel*(cos(phi_uf)-1) + tw/2*sin(phi_uf))*k_zt + (w_uf - r_wheel*(sin(phi_uf)*p_uf) + tw/2*cos(phi_uf)*p_uf)*k_zd;
      Fz_LR = mass_sprung*Constants.g*l_front/(2*l_total) + mass_unsprung_rear*Constants.g/2 + (z_ur + r_wheel*(cos(phi_ur)-1) - tw/2*sin(phi_ur))*k_zt + (w_ur - r_wheel*(sin(phi_uf)*p_uf) - tw/2*cos(phi_uf)*p_uf)*k_zd;
      Fz_RR = mass_sprung*Constants.g*l_front/(2*l_total) + mass_unsprung_rear*Constants.g/2 + (z_ur + r_wheel*(cos(phi_ur)-1) + tw/2*sin(phi_ur))*k_zt + (w_ur - r_wheel*(sin(phi_uf)*p_uf) + tw/2*cos(phi_uf)*p_uf)*k_zd;
    
      // Floor at zero (contact loss)
      //Fz_LF = max(0, Fz_LF);
      //Fz_RF = max(0, Fz_RF);
      //Fz_LR = max(0, Fz_LR);
      //Fz_RR = max(0, Fz_RR);
    
      // ===== Wheel-plane velocities (project body + roll rates) =====
      u_fl =  cos(delta_fl)*(u_t + tw/2*r_t) + sin(delta_fl)*(v_u + l_front*r_t - p_uf*(r_wheel - z_uf));
      v_fl = -sin(delta_fl)*(u_t + tw/2*r_t) + cos(delta_fl)*(v_u + l_front*r_t - p_uf*(r_wheel - z_uf));
      u_fr =  cos(delta_fr)*(u_t - tw/2*r_t) + sin(delta_fr)*(v_u + l_front*r_t - p_uf*(r_wheel - z_uf));
      v_fr = -sin(delta_fr)*(u_t - tw/2*r_t) + cos(delta_fr)*(v_u + l_front*r_t - p_uf*(r_wheel - z_uf));
      u_rl =  cos(delta_rl)*(u_t + tw/2*r_t) + sin(delta_rl)*(v_u - l_rear*r_t - p_ur*(r_wheel - z_ur));
      v_rl = -sin(delta_rl)*(u_t + tw/2*r_t) + cos(delta_rl)*(v_u - l_rear*r_t - p_ur*(r_wheel - z_ur));
      u_rr =  cos(delta_rr)*(u_t - tw/2*r_t) + sin(delta_rr)*(v_u - l_rear*r_t - p_ur*(r_wheel - z_ur));
      v_rr = -sin(delta_rr)*(u_t - tw/2*r_t) + cos(delta_rr)*(v_u - l_rear*r_t - p_ur*(r_wheel - z_ur));
    
      // ===== Dugoff-style friction reduction and mu =====
      vs_fl = u_t*sqrt(s_fl^2 + tan(alpha_fl)^2);
      vs_fr = u_t*sqrt(s_fr^2 + tan(alpha_fr)^2);
      vs_rl = u_t*sqrt(s_rl^2 + tan(alpha_rl)^2);
      vs_rr = u_t*sqrt(s_rr^2 + tan(alpha_rr)^2);
    
      mu_fl = max(0, mu0*(1 - As*vs_fl));
      mu_fr = max(0, mu0*(1 - As*vs_fr));
      mu_rl = max(0, mu0*(1 - As*vs_rl));
      mu_rr = max(0, mu0*(1 - As*vs_rr));
    
      // Non-dimensional force parameter κ and saturation fkappa
      kappa_fl = clip( mu_fl*Fz_LF*(1 - s_fl) / (2*sqrt((c_s*s_fl)^2 + (c_alpha*tan(alpha_fl))^2) + Constants.eps), 0, 1e4);
      kappa_fr = clip( mu_fr*Fz_RF*(1 - s_fr) / (2*sqrt((c_s*s_fr)^2 + (c_alpha*tan(alpha_fr))^2) + Constants.eps), 0, 1e4);
      kappa_rl = clip( mu_rl*Fz_LR*(1 - s_rl) / (2*sqrt((c_s*s_rl)^2 + (c_alpha*tan(alpha_rl))^2) + Constants.eps), 0, 1e4);
      kappa_rr = clip( mu_rr*Fz_RR*(1 - s_rr) / (2*sqrt((c_s*s_rr)^2 + (c_alpha*tan(alpha_rr))^2) + Constants.eps), 0, 1e4);
    
      fkappa_fl = if kappa_fl < 1 then kappa_fl*(2 - kappa_fl) else 1;
      fkappa_fr = if kappa_fr < 1 then kappa_fr*(2 - kappa_fr) else 1;
      fkappa_rl = if kappa_rl < 1 then kappa_rl*(2 - kappa_rl) else 1;
      fkappa_rr = if kappa_rr < 1 then kappa_rr*(2 - kappa_rr) else 1;
    
      // Tire forces
      Fx_LF = c_s*s_fl/(1 - s_fl)*fkappa_fl;
      Fx_RF = c_s*s_fr/(1 - s_fr)*fkappa_fr;
      Fx_LR = c_s*s_rl/(1 - s_rl)*fkappa_rl;
      Fx_RR = c_s*s_rr/(1 - s_rr)*fkappa_rr;
    
      Fy_LF = c_alpha*tan(alpha_fl)/(1 - s_fl)*fkappa_fl;
      Fy_RF = c_alpha*tan(alpha_fr)/(1 - s_fr)*fkappa_fr;
      Fy_LR = c_alpha*tan(alpha_rl)/(1 - s_rl)*fkappa_rl;
      Fy_RR = c_alpha*tan(alpha_rr)/(1 - s_rr)*fkappa_rr;
    
      // ===== Slip dynamics with relaxation lengths =====
      der(s_fl) = -abs(u_fl)/clip(tanh(abs(u_t)), 0.001, 1.0)/Lrelx*s_fl + (r_wheel*omega_fl - u_fl)/Lrelx;
      der(s_fr) = -abs(u_fr)/clip(tanh(abs(u_t)), 0.001, 1.0)/Lrelx*s_fr + (r_wheel*omega_fr - u_fr)/Lrelx;
      der(s_rl) = -abs(u_rl)/clip(tanh(abs(u_t)), 0.001, 1.0)/Lrelx*s_rl + (r_wheel*omega_rl - u_rl)/Lrelx;
      der(s_rr) = -abs(u_rr)/clip(tanh(abs(u_t)), 0.001, 1.0)/Lrelx*s_rr + (r_wheel*omega_rr - u_rr)/Lrelx;
    
      // clamp slip integration when outside bounds
      //when (pre(s_fl) >= s_max) or (pre(s_fl) <= s_min) then reinit(s_fl, sat(s_fl, s_min, s_max)); end when;
      //when (pre(s_fr) >= s_max) or (pre(s_fr) <= s_min) then reinit(s_fr, sat(s_fr, s_min, s_max)); end when;
      //when (pre(s_rl) >= s_max) or (pre(s_rl) <= s_min) then reinit(s_rl, sat(s_rl, s_min, s_max)); end when;
      //when (pre(s_rr) >= s_max) or (pre(s_rr) <= s_min) then reinit(s_rr, sat(s_rr, s_min, s_max)); end when;
    
      der(alpha_fl) = -abs(u_fl)*tan(alpha_fl)/Lrely - v_fl/Lrely;
      der(alpha_fr) = -abs(u_fr)*tan(alpha_fr)/Lrely - v_fr/Lrely;
      der(alpha_rl) = -abs(u_rl)*tan(alpha_rl)/Lrely - v_rl/Lrely;
      der(alpha_rr) = -abs(u_rr)*tan(alpha_rr)/Lrely - v_rr/Lrely;
    
      //when (pre(alpha_fl) >= alpha_max) or (pre(alpha_fl) <= alpha_min) then reinit(alpha_fl, sat(alpha_fl, alpha_min, alpha_max)); end when;
      //when (pre(alpha_fr) >= alpha_max) or (pre(alpha_fr) <= alpha_min) then reinit(alpha_fr, sat(alpha_fr, alpha_min, alpha_max)); end when;
      //when (pre(alpha_rl) >= alpha_max) or (pre(alpha_rl) <= alpha_min) then reinit(alpha_rl, sat(alpha_rl, alpha_min, alpha_max)); end when;
      //when (pre(alpha_rr) >= alpha_max) or (pre(alpha_rr) <= alpha_min) then reinit(alpha_rr, sat(alpha_rr, alpha_min, alpha_max)); end when;
    
      // ===== Wheel spin dynamics (motor → wheel, road torque) =====
      der(omega_fl) = ( torqueGain*(omega_m/gratio - omega_fl) - r_wheel*Fx_LF )/Iyy_wheel;
      der(omega_fr) = ( torqueGain*(omega_m/gratio - omega_fr) - r_wheel*Fx_RF )/Iyy_wheel;
      der(omega_rl) = ( torqueGain*(omega_m/gratio - omega_rl) - r_wheel*Fx_LR )/Iyy_wheel;
      der(omega_rr) = ( torqueGain*(omega_m/gratio - omega_rr) - r_wheel*Fx_RR )/Iyy_wheel;
    
      // ===== Suspension kinematics for spring/aux/bump & SU lateral link =====
      z_SLF = (h_s - r_wheel + z_uf - z_s)/cos(phi_s) - h_s + r_wheel + l_front*theta_s + (phi_s - phi_uf)*(tw/2);
      z_SRF = (h_s - r_wheel + z_uf - z_s)/cos(phi_s) - h_s + r_wheel + l_front*theta_s - (phi_s - phi_uf)*(tw/2);
      z_SLR = (h_s - r_wheel + z_ur - z_s)/cos(phi_s) - h_s + r_wheel - l_rear*theta_s + (phi_s - phi_ur)*(tw/2);
      z_SRR = (h_s - r_wheel + z_ur - z_s)/cos(phi_s) - h_s + r_wheel - l_rear*theta_s - (phi_s - phi_ur)*(tw/2);
    
      zdot_SLF = w_uf - w_s + l_front*q_s + (p_s - p_uf)*(tw/2);
      zdot_SRF = w_uf - w_s + l_front*q_s - (p_s - p_uf)*(tw/2);
      zdot_SLR = w_ur - w_s - l_rear*q_s + (p_s - p_ur)*(tw/2);
      zdot_SRR = w_ur - w_s - l_rear*q_s - (p_s - p_ur)*(tw/2);
    
      // Bump stops
      F_BSLF = if abs(z_SLF) <= h_bs then 0 else (-z_SLF + sign(z_SLF)*h_bs)*k_bs;
      F_BSRF = if abs(z_SRF) <= h_bs then 0 else (-z_SRF + sign(z_SRF)*h_bs)*k_bs;
      F_BSLR = if abs(z_SLR) <= h_bs then 0 else (-z_SLR + sign(z_SLR)*h_bs)*k_bs;
      F_BSRR = if abs(z_SRR) <= h_bs then 0 else (-z_SRR + sign(z_SRR)*h_bs)*k_bs;
    
      // Squat/lift lateral components & aux roll bars
      F_SQLF = (k_slf + z_SLF/l_saf)*(  Fy_LF*cos(phi_s) - Fz_LF*sin(phi_s)) - (min(Fx_LF,0)*k_sadf + max(Fx_LF,0)*k_sad2f);
      F_SQRF = (k_slf + z_SRF/l_saf)*( -Fy_RF*cos(phi_s) + Fz_RF*sin(phi_s)) - (min(Fx_RF,0)*k_sadf + max(Fx_RF,0)*k_sad2f);
      F_SQLR = (k_slr + z_SLR/l_sar)*(  Fy_LR*cos(phi_s) - Fz_LR*sin(phi_s)) + (min(Fx_LR,0)*k_sadr + max(Fx_LR,0)*k_sad2r);
      F_SQRR = (k_slr + z_SRR/l_sar)*( -Fy_RR*cos(phi_s) + Fz_RR*sin(phi_s)) + (min(Fx_RR,0)*k_sadr + max(Fx_RR,0)*k_sad2r);
    
      F_SLF = mass_sprung*Constants.g*l_rear/(2*l_total) - z_SLF*k_sf - zdot_SLF*k_sdf + (phi_s - phi_uf)*k_tsf/tw_f + F_BSLF + F_SQLF;
      F_SRF = mass_sprung*Constants.g*l_rear/(2*l_total) - z_SRF*k_sf - zdot_SRF*k_sdf - (phi_s - phi_uf)*k_tsf/tw_f + F_BSRF + F_SQRF;
      F_SLR = mass_sprung*Constants.g*l_front/(2*l_total) - z_SLR*k_sr - zdot_SLR*k_sdr + (phi_s - phi_ur)*k_tsr/tw_r + F_BSLR + F_SQLR;
      F_SRR = mass_sprung*Constants.g*l_front/(2*l_total) - z_SRR*k_sr - zdot_SRR*k_sdr - (phi_s - phi_ur)*k_tsr/tw_r + F_BSRR + F_SQRR;
    
      // SU lateral compliant pin links
      // delta_yf, delta_yr and their rates:
      // (match MATLAB expressions)
      F_RAF = ( ((h_s - r_wheel + z_uf - z_s)*tan(phi_s) - (delta_y_suf))*cos(phi_s) - (h_raf - r_wheel)*sin(phi_s - phi_uf) )*k_ras
              + ( ((h_s - r_wheel + z_uf - z_s)*cos(phi_s) - (delta_y_suf)*sin(phi_s))*p_s
                  + (w_uf - w_s)*sin(phi_s) - delta_v_suf*cos(phi_s)
                  - (h_raf - r_wheel)*cos(phi_s - phi_uf)*(p_s - p_uf) )*k_rad;
    
      F_RAR = ( ((h_s - r_wheel + z_ur - z_s)*tan(phi_s) - (delta_y_sur))*cos(phi_s) - (h_rar - r_wheel)*sin(phi_s - phi_ur) )*k_ras
              + ( ((h_s - r_wheel + z_ur - z_s)*cos(phi_s) - (delta_y_sur)*sin(phi_s))*p_s
                  + (w_ur - w_s)*sin(phi_s) - delta_v_sur*cos(phi_s)
                  - (h_rar - r_wheel)*cos(phi_s - phi_ur)*(p_s - p_ur) )*k_rad;
    
      // ===== Planar chassis forces/accels =====
      a_x = ( (Fx_LF*cos(delta_fl) - Fy_LF*sin(delta_fl)) + (Fx_RF*cos(delta_fr) - Fy_RF*sin(delta_fr))
            + (Fx_LR*cos(delta_rl) - Fy_LR*sin(delta_rl)) + (Fx_RR*cos(delta_rr) - Fy_RR*sin(delta_rr)) )/mass_total;
    
      rdot = ( + tw/2*(Fx_LF*cos(delta_fl) - Fy_LF*sin(delta_fl)) - tw/2*(Fx_RF*cos(delta_fr) - Fy_RF*sin(delta_fr))
               + tw/2*(Fx_LR*cos(delta_rl) - Fy_LR*sin(delta_rl)) - tw/2*(Fx_RR*cos(delta_rr) - Fy_RR*sin(delta_rr))
               + l_front*(Fx_LF*sin(delta_fl) + Fy_LF*cos(delta_fl)) + l_front*(Fx_RF*sin(delta_fr) + Fy_RF*cos(delta_fr))
               - l_rear *(Fx_LR*sin(delta_rl) + Fy_LR*cos(delta_rl)) - l_rear *(Fx_RR*sin(delta_rr) + Fy_RR*cos(delta_rr)) )/Izz_t;
    
      // Yaw–roll coupling (uses previous phiddot via algebraic; we couple directly as in MATLAB)
      psiddot = -Ixz_s/Izz_t*phiddot_s + rdot;
    
      a_y_uf = ( (Fx_LF*sin(delta_fl) + Fy_LF*cos(delta_fl)) + (Fx_RF*sin(delta_fr) + Fy_RF*cos(delta_fr))
               - F_RAF*cos(phi_s) - (F_SLF + F_SRF)*sin(phi_s) )/mass_unsprung_front;
    
      a_y_ur = ( (Fx_LR*sin(delta_rl) + Fy_LR*cos(delta_rl)) + (Fx_RR*sin(delta_rr) + Fy_RR*cos(delta_rr))
               - F_RAR*cos(phi_s) - (F_SLR + F_SRR)*sin(phi_s) )/mass_unsprung_rear;
    
      a_y_s  = ( + F_RAF*cos(phi_s) + F_RAR*cos(phi_s)
               + (F_SLF + F_SRF + F_SLR + F_SRR)*sin(phi_s) )/mass_sprung;
    
      // ===== Kinematics & Dynamics Integration =====
      der(x_t)   =  u_t*cos(psi_t) - v_u*sin(psi_t);
      der(u_t)   =  v_u*r_t + a_x;
      der(psi_t) =  r_t;
      der(r_t)   =  psiddot;
      der(y_u)   =  u_t*sin(psi_t) + v_u*cos(psi_t);
      der(v_u)   = -u_t*r_t + (mass_unsprung_front*a_y_uf + mass_unsprung_rear*a_y_ur)/(mass_unsprung_front + mass_unsprung_rear);
    
      der(delta_y_suf) = delta_v_suf;
      der(delta_v_suf) = a_y_s + l_front*psiddot - a_y_uf;
      der(delta_y_sur) = delta_v_sur;
      der(delta_v_sur) = a_y_s - l_rear*psiddot - a_y_ur;
    
      der(z_uf) = w_uf;
      der(w_uf) = Constants.g - (Fz_LF + Fz_RF + F_RAF*sin(phi_s) - (F_SLF + F_SRF)*cos(phi_s))/mass_unsprung_front;
    
      der(z_ur) = w_ur;
      der(w_ur) = Constants.g - (Fz_LR + Fz_RR + F_RAR*sin(phi_s) - (F_SLR + F_SRR)*cos(phi_s))/mass_unsprung_rear;
    
      der(z_s)  = w_s;
      der(w_s)  = Constants.g - ( -(F_RAF + F_RAR)*sin(phi_s) + (F_SLF + F_SRF + F_SLR + F_SRR)*cos(phi_s) )/mass_sprung;
    
      der(phi_uf) = p_uf;
      der(p_uf)   = ( + F_SRF*(tw/2) - F_SLF*(tw/2) - F_RAF*(h_raf - r_wheel)
                      + Fz_LF*( r_wheel*sin(phi_uf) + tw/2*cos(phi_uf) - k_lt*Fy_LF )
                      - Fz_RF*( -r_wheel*sin(phi_uf) + tw/2*cos(phi_uf) + k_lt*Fy_RF )
                      - (Fx_LF*sin(delta_fl) + Fy_LF*cos(delta_fl))*(r_wheel - z_uf)
                      - (Fx_RF*sin(delta_fr) + Fy_RF*cos(delta_fr))*(r_wheel - z_uf) )/Ixx_uf;
    
      der(phi_ur) = p_ur;
      der(p_ur)   = ( + F_SRR*(tw/2) - F_SLR*(tw/2) - F_RAR*(h_rar - r_wheel)
                      + Fz_LR*( r_wheel*sin(phi_ur) + tw/2*cos(phi_ur) - k_lt*Fy_LR )
                      - Fz_RR*( -r_wheel*sin(phi_ur) + tw/2*cos(phi_ur) + k_lt*Fy_RR )
                      - (Fx_LR*sin(delta_rl) + Fy_LR*cos(delta_rl))*(r_wheel - z_ur)
                      - (Fx_RR*sin(delta_rr) + Fy_RR*cos(delta_rr))*(r_wheel - z_ur) )/Ixx_ur;
    
      der(phi_s)  = p_s;
    
      pdot_s      = ( + F_SLF*(tw/2) + F_SLR*(tw/2) - F_SRF*(tw/2) - F_SRR*(tw/2)
                       - F_RAF/cos(phi_s)*(h_s - z_s - r_wheel + z_uf - (h_raf - r_wheel)*cos(phi_uf))
                       - F_RAR/cos(phi_s)*(h_s - z_s - r_wheel + z_ur - (h_rar - r_wheel)*cos(phi_ur)) ) / (10*Ixx_s);
      phiddot_s   = -Ixz_s/Ixx_s*psiddot + pdot_s;
      der(p_s)    = phiddot_s;
    
      der(theta_s)= q_s;
      der(q_s)    = ( + l_front*(F_SLF + F_SRF) - l_rear*(F_SLR + F_SRR)
                      + ( (Fx_LF*cos(delta_fl) - Fy_LF*sin(delta_fl))
                        + (Fx_RF*cos(delta_fr) - Fy_RF*sin(delta_fr))
                        + (Fx_LR*cos(delta_rl) - Fy_LR*sin(delta_rl))
                        + (Fx_RR*cos(delta_rr) - Fy_RR*sin(delta_rr)) )*(h_s - z_s) )/Iyy_s;
                        
      // compute mechanical power
      Pmech = Kt_q*Iq*omega_m;
      Ploss = Iq^2*R_phi;
  
      // accelerometer specific force
      C_n2b = transpose(eul2rot({phi_s, theta_s, mod(psi_t + Constants.PI, 2*Constants.PI) - Constants.PI}));
      for i in 1:3 loop
        specific_g[i] = C_n2b[i,3]*Constants.g;
      end for;
  
      // emi model inputs
      emi.Iq = Iq;
      emi.lambda = lambda_m;
      mag.phi = phi_s;
      mag.theta = theta_s;
      mag.psi = psi_t;
      mx = mag.b_earth[1]+0*emi.b_wire[1];
      my = mag.b_earth[2]+0*emi.b_wire[2];
      mz = mag.b_earth[3]+0*emi.b_wire[3]; 
      
  // rollover check
      when delta_f <> 0 then
        turn_radius = clip(abs(l_total/tan(delta_f)),0,1000);
      elsewhen delta_f == 0 then
        turn_radius = 1000;
      end when;
      
      when Fz_LF <= 0.5 or Fz_RF <= 0.5 or Fz_LR <= 0.5 or Fz_RR <= 0.5 then
        rollover_detected = 1;
      end when;
      
      when tau_shaft > tau_yield then
        shaft_failure = 1;
      end when;
      
      rollover_metric_roll = phi_s;
      rollover_metric_z = -z_s;
      
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
