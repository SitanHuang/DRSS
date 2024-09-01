classdef NamedObj < handle
    properties
        % name of this obj
        label
    end

    methods
        function obj = NamedObj(varargin)
            obj = DRSS.util.objectAssign(obj, [{"label"} varargin], 1);
        end
    end
end
