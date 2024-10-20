function [this, sys, ss0] = resetTransientData(this, sys, ss0)
  [this, sys, ss0] = resetTransientData@DRSS.core.dynamics.Dynamics(this, sys, ss0);

  ss0.declareCustomParam("CDr");
  ss0.declareCustomParam("CAr");
  ss0.declareCustomParam("SSM");

  this.recalcTransientParameters(sys);
end