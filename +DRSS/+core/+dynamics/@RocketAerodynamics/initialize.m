function this=initialize(this, inputs)
    if mod(length(inputs), 2) ~= 0
      error('Arguments must be provided as key-value pairs.');
    end

    aerodynamicProfile = struct();

    for i = 1:2:length(inputs)
      key = inputs{i};
      value = inputs{i+1};

      if ischar(key) || isstring(key)
        key = char(key);
      else
        error('Property names must be strings or character arrays.');
      end

      aerodynamicProfile.(key) = value;
    end

    % Define the list of required properties with descriptions
    requiredProps = {
      'D',          'Rocket diameter [m]'
      % Know from sys obj:
      % 'L',          'Entire rocket length [m]'
      'L_nose',     'Nose cone length [m]'
      % Know from sys obj:
      % 'L_body',     'Rocket body total length [m]'
      % Know from D & sys obj:
      % 'A_planform', 'Rocket body projected side area not including fins [m^2]'
      'l_tail',     'Length to boat tail from nose tip [m]'
      'L_tail_c',   'Boat tail curved section length [m]'
      'L_tail_f',   'Boat tail flat section length [m]'
      'L_tail_rr',  'Retaining ring length from end of boat tail [in]'
      'D_tail',     'Boat tail aft diameter [m]'
      'D_tail_rr',  'Retaining ring diameter [in]'
      'A_fin',      'Fin area [m^2]'
      'N_fins',     'Number of fins []'
      'A_fin_e',    'Extended fin area [m^2]'
      'l_fin',      'Length to tip of fins from tip of nose [m]'
      't_fin',      'Fin thickness [m]'
      's',          'Fin semispan [m]'
      'cr',         'Fin root chord length [m]'
      'cm',         'Fin mid chord length [m]'
      'ct',         'Fin tip chord length [m]'
      % 'L_fin_foil', 'Length to tip of aux fins from tip of nose cone [m]'
      % 't_fin_foil', 'Air foil thickness [m]'
      % 'cr_foil',    'Aux airfoil fins root chord length [m]'
      % 'ct_foil',    'Aux airfoil fins tip chord length [m]'
      % 's_foil',     'Aux airfoil fins semispan [m]'
      % A = (pi / 4) * D^2 + A_nacelle_projected*n_nacelle
      % 'A',          'Rocket cross-sectional area [m^2]'
      'CDr',        'Reference drag coefficient [1]'
      'CP',         'Constant rocket CP location [m]'
    };

    propNames = requiredProps(:,1);
    propDescriptions = requiredProps(:,2);

    % Check for missing required properties
    providedProps = fieldnames(aerodynamicProfile);
    missingProps = setdiff(propNames, providedProps);

    if ~isempty(missingProps)
      missingMessages = cell(length(missingProps),1);
      for i = 1:length(missingProps)
        idx = strcmp(propNames, missingProps{i});
        description = propDescriptions{idx};
        missingMessages{i} = sprintf('%s: %s', missingProps{i}, description);
      end

      error( ...
        "RocketAerodynamics Constructor Error: Missing required properties: \n%s", ...
        sprintf('  - %s\n', missingMessages{:}) ...
      );
    end

    this.aerodynamicProfile = aerodynamicProfile;
end