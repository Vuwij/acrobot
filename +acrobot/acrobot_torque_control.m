classdef acrobot_torque_control < matlab.System

    % Public, tunable properties
    properties
        steps_per_rotation = 2797;
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        a;
        encoder;
    end
    
    methods(Access = public)
        function obj = acrobot_torque_control()
            obj.a = arduino('/dev/ttyUSB0','Nano3','Libraries',{'RotaryEncoder', 'I2C'});
            obj.encoder = rotaryEncoder(obj.a,'D2','D3',obj.steps_per_rotation);
        end
        
        function calculateMotorCurves(obj)

            %% Count Encoder
            pwm_step = 2;
            trials = 10;
            pwms = -1:pwm_step:1;
            pwm_values = cell(1,length(pwms));
            tstep = 0.01;
            time_duration = 0:tstep:0.4;
            
            for j = 1:1:length(pwms)                
                for k = 1:1:trials
                    if (pwms(j) > 0)
                        writeDigitalPin(obj.a, 'D6', 1);
                        writeDigitalPin(obj.a, 'D7', 0);
                    else
                        writeDigitalPin(obj.a, 'D6', 0);
                        writeDigitalPin(obj.a, 'D7', 1);
                    end
                    % Stop the movement
                    writePWMDutyCycle(obj.a,'D9',0);
                    pause(2.0);
                    obj.encoder.resetCount();

                    % Record the speed increase
                    rate = rateControl(1/tstep);
                    position = zeros(1, length(duration));
                    power = min(abs(pwms(j)),1);
                    writePWMDutyCycle(obj.a,'D9',power);
                    for i = 1:1:length(time_duration)
                        position(i) = readCount(obj.encoder) / obj.steps_per_rotation;
                        pwm_values{j,k} = position;
                        waitfor(rate);
                    end
                end
            end
            
            writePWMDutyCycle(obj.a,'D9',0);
            
            hold off;
            for j = 1:1:length(pwms)
                p_avg = zeros(1,length(pwms));
                for k = 1:1:trials
                    p_avg = p_avg + pwm_values{j,k};
                end
                p = spline(time_duration, p_avg);
                v = fnder(p,1);
                a = fnder(p,2);
                
                subplot(3,1,1);
                plot(time_duration, ppval(p, time_duration))
                hold on;
                grid minor;
                xlabel('Time t');
                ylabel('Position r/s');
                title('Position vs time graph');                

                subplot(3,1,2);
                plot(time_duration, ppval(v, time_duration))
                hold on;
                grid minor;
                xlabel('Time t');
                ylabel('Position r/s');
                title('Speed vs time graph');      
                
                subplot(3,1,3);
                plot(time_duration, ppval(a, time_duration))
                hold on;
                grid minor;
                xlabel('Time t');
                ylabel('Position r/s');
                title('Acceleration vs time graph');
                
            end
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function pwm_ratio = stepImpl(obj, tau, tau_in)
            pwm_ratio = 1; % Between -1 and 1, where 1 is maximum speed, 0 is mininum speed
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end