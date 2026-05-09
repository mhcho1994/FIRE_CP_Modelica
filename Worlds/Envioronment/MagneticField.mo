within FIRE_CP_Modelica_Update.Worlds.Envioronment;

model MagneticField
  parameter Real magneticField0[3] = {21.0e-6, 0.0, 43.0e-6}
    "Nominal world-frame magnetic field [T]";
  Modelica.Blocks.Interfaces.RealOutput magneticField[3];
equation
  magneticField = magneticField0;
end MagneticField;
