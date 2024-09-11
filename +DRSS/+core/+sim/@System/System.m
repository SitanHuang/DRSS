classdef System < DRSS.core.obj.MassGroup
    %SYSTEM A System is a special type of MassGroup that can be subject to
    %   certain dynamics and holds states data in the time domain
    %
    %   A System obj is a **handle** -> copying a System (either directly or
    %   when passing as value in function calls) creates another reference to
    %   the same object.
    %
    %   A System is much more accurate in calculating Moment of Inertia than
    %   MassGroup as it calculates using the individual sections rather than
    %   using the System's cgX and mass

    properties
        dynamicsList = []

        systemState = DRSS.core.sim.SystemState()

        IAarray = {}
    end

    methods
        % function obj=overrideCGX(obj, cgX) %#ok<INUSD>
        %     error("Overrides are not allowed for a System obj")
        % end
        % function obj=overrideM(obj, m) %#ok<INUSD>
        %     error("Overrides are not allowed for a System obj")
        % end
        % function obj=overrideLen(obj, len) %#ok<INUSD>
        %     error("Overrides are not allowed for a System obj")
        % end
        % function obj=overrideSideArea(obj, sideArea) %#ok<INUSD>
        %     error("Overrides are not allowed for a System obj")
        % end
    end

    methods
        function this = subjecTo(this, dynamics)
            this.dynamicsList = [this.dynamicsList {dynamics}];
        end

        function momentOfInertia=momentOfInertia(this)
            momentOfInertia = 0;

            lengthCursor = 0;

            this.IAarray = {};

            for i = 1:length(this.massList)
                currentMass = this.massList{i};

                I = currentMass.calcMomentOfInertia( ...
                    this.cgX, ... % vehicle CG
                    lengthCursor + currentMass.cgX + currentMass.offset ... % length from nose cone to section cg
                );

                this.IAarray{i} = {currentMass.label, 'I',I,'cg0',this.cgX,'l_cg_i',lengthCursor + currentMass.cgX + currentMass.offset};

                lengthCursor = lengthCursor + currentMass.len + currentMass.offset;

                momentOfInertia = momentOfInertia + I;
            end
        end
    end
end
