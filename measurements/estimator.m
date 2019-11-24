classdef estimator < handle
   properties
      imu;
      encoder;
      pitch;
      motor_angle;
   end
   methods
      function obj = estimator(val1, val2)
        if nargin == 1
            obj.imu = val1;
            obj.encoder = val2;
        end
       end
      function  p = imuUpdate(obj)
          imuf = imufilter();
          [accel, gyro] = obj.imu();
           q = imuf(accel, gyro);
           rpy = quat2eul(q);
           rpy = rad2deg(rpy);
           p = rpy(3);
           obj.pitch = p;
      end
      function motor_angle = motorEncoderUpdate(obj)
         count = readCount(obj.encoder);
         cycles = 9;
         count_per_revolution = abs(count) / cycles;
         angle = count * count_per_revolution;
         if (abs(angle) >= 360) 
            if (angle < 0) 
                angle = angle + 360;
            
            elseif (angle > 0) 
                angle = angle - 360;
            end 
         end
         motor_angle = angle;
         obj.motor_angle = angle;
      end
   end
end