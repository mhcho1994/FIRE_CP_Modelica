within FIRE_CP_Modelica_Update.Interfaces;

expandable connector CommandBus
  Real steering "Normalized steering command [-1, 1]";
  Real throttle "Normalized throttle command [-1, 1]";
  Real leftElevon "Normalized left elevon command [-1, 1]";
  Real rightElevon "Normalized right elevon command [-1, 1]";
end CommandBus;
