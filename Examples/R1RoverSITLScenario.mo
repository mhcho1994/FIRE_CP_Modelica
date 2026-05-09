within FIRE_CP_Modelica_Update.Examples;

model R1RoverSITLScenario
  Worlds.World world;
  Vehicles.Rover.R1Rover rover;
  Interfaces.PwmBus pwm;

  parameter Real throttlePwm = 1700;
  parameter Real steeringPwmStraight = 1500;
  parameter Real steeringPwmTurn = 1700;
  parameter Real turnStart = 5.0;

equation
  connect(world.world, rover.world);
  connect(pwm, rover.pwm);

  pwm.ch_0 = if time < turnStart then steeringPwmStraight else steeringPwmTurn;
  pwm.ch_2 = throttlePwm;

  annotation(experiment(StartTime = 0, StopTime = 30, Tolerance = 1e-6, Interval = 0.02));
end R1RoverSITLScenario;
