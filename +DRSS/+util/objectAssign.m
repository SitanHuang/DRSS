function obj = objectAssign(obj, listVarArgin, startIndex)
    arguments
        obj
        listVarArgin
        startIndex (1,1) {mustBeInteger, mustBePositive} =1
    end

    for k = (startIndex):2:length(listVarArgin)
        if isprop(obj, listVarArgin{k})
            obj.(listVarArgin{k}) = listVarArgin{k+1};
        else
            error('Invalid property');
        end
    end
end