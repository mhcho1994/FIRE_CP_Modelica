within FIRE_CP_Modelica_Update.Interfaces;

expandable connector WorldBus
  Real gravity[3] "Gravity vector in world frame [m/s2]";
  Real magneticField[3] "Magnetic field in world frame [T]";
  Real wind[3] "Wind velocity in world frame [m/s]";
  Real temperature "Atmospheric temperature [K]";
  Real pressure "Atmospheric pressure [Pa]";
  Real density "Atmospheric density [kg/m3]";
end WorldBus;
