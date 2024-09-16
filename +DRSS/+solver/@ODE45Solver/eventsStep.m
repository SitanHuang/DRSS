function [value, isterminal, direction] = eventsStep(this, t, states, system)
    value = 1;
    if system.systemState.terminate
      value = 0;
    end
    isterminal = 1;
    direction = 0;
end
