function mg = recalcMassGroupProperties(mg)
    % Initialize sums
    totalMass = 0;
    totalLength = 0;
    totalSideArea = 0;
    weightedCgXSum = 0;

    % Iterate over all mass objects in the list
    for i = 1:length(mg.massList)
        currentMass = mg.massList{i};

        % Sum masses
        totalMass = totalMass + currentMass.m;

        % Weighted sum for center of gravity calculation before sum len
        weightedCgXSum = weightedCgXSum + currentMass.m * (currentMass.cgX + totalLength + currentMass.offset);

        % Sum lengths
        totalLength = totalLength + currentMass.len + currentMass.offset;

        % Sum side areas
        totalSideArea = totalSideArea + currentMass.sideArea;
    end

    % Calculate new properties
    mg.m = totalMass;
    mg.len = totalLength;

    % Calculate the new center of gravity relative to the start of the first mass object
    if totalMass > 0
        mg.cgX = weightedCgXSum / totalMass;
    else
        mg.cgX = 0; % Handle the case where total mass is zero
    end

    mg.sideArea = totalSideArea;

    if ~isempty(mg.overriddenCGX)
        mg.cgX = mg.overriddenCGX;
    end
    if ~isempty(mg.overriddenM)
        mg.m = mg.overriddenM;
    end
    if ~isempty(mg.overriddenLen)
        mg.len = mg.overriddenLen;
    end
    if ~isempty(mg.overriddenSideArea)
        mg.sideArea = mg.overriddenSideArea;
    end
end
