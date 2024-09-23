function [this, sys, ss0] = resetTransientData(this, sys, ss0)
  [this, sys, ss0] = resetTransientData@DRSS.core.dynamics.Dynamics(this, sys, ss0);


  this.recalcTransientParameters(sys);
end