classdef MassGroup < DRSS.core.obj.Mass
  % MASSGROUP Abstract representation of a serial, 1D arrangement of Mass objs
  %   and any of their derivatives (e.g., another MassGroup)

  properties
    massList = []
  end

  methods
    function mg = appendChild(mg, child)
      mg.massList = [mg.massList {child}];

      mg = mg.recalcMassGroupProperties();
    end

    function obj=overrideCGX(obj, cgX)
      obj.cgX = cgX;
    end
    function obj=overrideM(obj, m)
      obj.m = m;
    end
    function obj=overrideLen(obj, len)
      obj.len = len;
    end
    function obj=overrideSideArea(obj, sideArea)
      obj.sideArea = sideArea;
    end

    function this=setInertialGeometryRecursive(this, geometry)
      this.setInertialGeometry(geometry);

      for i = 1:length(this.massList)
        child = this.massList{i};

        if isa(geometry, "DRSS.core.obj.MassGroup")
          child.setInertialGeometryRecursive(geometry);
        else
          child.setInertialGeometry(geometry);
        end
      end
    end
  end

  methods
    % This method sums up all the child masses to the properties of
    % the MassGroup, which now acts as a single Mass obj.
    %
    % CAUTION: This method overrides the mass set by overrideMass
    %
    % This method is called automatically on appendChild()
    mg = recalcMassGroupProperties(mg);
  end
end