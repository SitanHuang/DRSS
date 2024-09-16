classdef Mass < DRSS.core.obj.NamedObj
    %MASS Abstract representation of a one-dimensional mass span which lies
    %   on a horizontal axis
    %
    % Example: Mass("Mass Label", "m", 123.12, "cgX", 0)

    properties
        % mass (g)
        m (1,1)
        % length of this 1D mass (m)
        len (1,1)
        % offset in length from the end of the previous Mass in a MassGroup
        offset (1,1) = 0

        % CG from the origin of the mass' local 1D coordinate system
        cgX (1,1) = 0

        % cross-sectional area of this mass projected to rocket's side (m^2)
        sideArea (1,1) = 0

        momentOfInertiaMethod = DRSS.core.obj.MomentOfInertiaMethod.HollowCylinderApproximation;
        momentOfInertiaGeometry = struct

        lockedGeometry = false
    end

    methods
        function obj=setCGX(obj, cgX)
            obj.cgX = cgX;
        end
        function obj=setM(obj, m)
            obj.m = m;
        end
        function obj=setLen(obj, len)
            obj.len = len;
        end
        function obj=setOffset(obj, offset)
            obj.offset = offset;
        end
        function obj=setSideArea(obj, sideArea)
            obj.sideArea = sideArea;
        end
        function this=setInertialGeometry(this, geometry)
            if ~this.lockedGeometry
                this.momentOfInertiaGeometry = geometry;
            end
        end
        function this=setMomentOfInertiaMethod(this, method)
            this.momentOfInertiaMethod = method;
        end
        function this=lockGeometry(this)
            this.lockedGeometry = true;
        end

        function momentOfInertia=momentOfInertia(obj)
            momentOfInertia = obj.calcMomentOfInertia(0, 0);
        end

        % The moment of inertia about the axis perpendicular to the length of
        % this Mass with respect to some refCG (e.g., vehicle's CG) and the
        % absolute CG of this Mass with respect to the whole system
        function I = calcMomentOfInertia(mass, refCG, massCGAbs)
            M = mass.m;
            L = mass.len;

            if mass.momentOfInertiaMethod == DRSS.core.obj.MomentOfInertiaMethod.HollowCylinderApproximation
                Ro = mass.momentOfInertiaGeometry.Ro;
                Ri = mass.momentOfInertiaGeometry.Ri;

                % INPUT(S)= M - mass of section [m]
                %           L - length of section [m]
                %           CG0 - vehicle CG [m]
                %           l_CG_i - length from nose cone to section CG [m]
                %           Ro_i - outer radius of section [m]
                %           Ri_i - inner radius of section [m]

                I = (M / 6) * (L^2 + 6 * (refCG - massCGAbs)^2 + (3 * (Ro^4 - Ri^4) / (Ro^2 - Ri^2)));
            elseif mass.momentOfInertiaMethod == DRSS.core.obj.MomentOfInertiaMethod.SolidCylinderApproximation
                R = mass.momentOfInertiaGeometry.R;

                I = M / 24 * (4 * L^2 + 24 * (refCG - massCGAbs)^2 + 3 * (R * 2)^2);
            else
                error('Unsupported momentOfInertiaMethod');
            end
        end
    end
end
