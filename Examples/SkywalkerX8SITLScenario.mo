within FIRE_CP_Modelica_Update.Examples;

model SkywalkerX8SITLScenario
  Worlds.World world;
  Vehicles.FixedWing.SkywalkerX8 aircraft;
  Interfaces.PwmBus pwm;

  parameter Real leftElevonPwm = 1500;
  parameter Real rightElevonPwm = 1500;
  parameter Real throttlePwm = 1650;

equation
  connect(world.world, aircraft.world);
  connect(pwm, aircraft.pwm);

  pwm.ch_0 = leftElevonPwm;
  pwm.ch_1 = rightElevonPwm;
  pwm.ch_2 = throttlePwm;

  annotation(experiment(StartTime = 0, StopTime = 30, Tolerance = 1e-6, Interval = 0.02));
end SkywalkerX8SITLScenario;
