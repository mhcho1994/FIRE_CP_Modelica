within FIRE_CP_Modelica_Update.Worlds.Envioronment;

model Gravity
  parameter Real g = FIRE_CP_Modelica_Update.Utilities.Constants.g
    "Gravity magnitude [m/s2]";
  Modelica.Blocks.Interfaces.RealOutput gravity[3];
equation
  gravity = {0, 0, -g};
end Gravity;
