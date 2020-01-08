classdef acrobot_torque_out < matlab.System

    % Public, tunable properties
    properties

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function pwm_ratio = stepImpl(obj, tau, tau_in)
            pwm_ratio = 1; % Between 1 and 0, where 1 is maximum speed, 0 is mininum speed
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
