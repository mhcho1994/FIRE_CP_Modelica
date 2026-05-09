within FIRE_CP_Modelica_Update.Interfaces;

expandable connector SensorBus
  Real x "World x position [m]";
  Real y "World y position [m]";
  Real z "World z position [m]";
  Real vx "World x velocity [m/s]";
  Real vy "World y velocity [m/s]";
  Real vz "World z velocity [m/s]";
  Real u "Body forward velocity [m/s]";
  Real v "Body lateral velocity [m/s]";
  Real w "Body vertical velocity [m/s]";
  Real ax "Body x accelerometer specific force [m/s2]";
  Real ay "Body y accelerometer specific force [m/s2]";
  Real az "Body z accelerometer specific force [m/s2]";
  Real phi "Roll [rad]";
  Real theta "Pitch [rad]";
  Real psi "Yaw [rad]";
  Real p "Roll rate [rad/s]";
  Real q "Pitch rate [rad/s]";
  Real r "Yaw rate [rad/s]";
  Real mx "Body x magnetic field [T]";
  Real my "Body y magnetic field [T]";
  Real mz "Body z magnetic field [T]";
  Real pressure "Barometric pressure [Pa]";
  Real temperature "Barometric temperature [K]";
  Real baroAltitude "Barometric altitude [m]";
  Real baroClimbRate "Barometric climb rate [m/s]";
end SensorBus;
