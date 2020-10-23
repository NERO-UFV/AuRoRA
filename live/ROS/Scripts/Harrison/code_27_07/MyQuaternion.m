classdef MyQuaternion
    % Create a quaternion Class
    properties (Access = public)
        
        % Elements of the Quaternion
        w   % Real Part
        i   % First Imaginary Part
        j   % Second Imaginary Part
        k   % third Imaginary Part
         
    end
    
    methods
        % Construction of the quaternion
        function obj = MyQuaternion(q1, q2, q3, q4)
        obj.w = q1;
        obj.i = q2;
        obj.j = q3;
        obj.k = q4;
        end
        
        % Returns the quaternion constructed
        function q = quaternion(self)
            q = [self.w, self.i, self.j, self.k];
        end
        
        % Returns its real part
        function real_part = real(self)
           real_part = self.w;
        end
           
        % Returns its imaginary part
        function imaginary_part = imag(self)
           imaginary_part = [self.i, self.j, self.k];
        end
        
        % Returns its norm
        function quaternion_norm = norm(q)
           quaternion_norm = sqrt(sum(q.quaternion .^(2)));
        end
        
        % Returns the quaternion normalized
        function quaternion_normalized = normalized(q)
           quaternion_normalized = q.quaternion/sqrt(sum(q.quaternion .^(2)));
        end
        
        % Returns its angle (argument) in rad
        function argument = arg(self)
           top = sign(self.k) * sqrt(self.i^(2) + self.j^(2) + self.k^(2));
           bottom = self.w;
           argument = atan2(top, bottom);
        end
        
        % Returns its rotation axis
        function axis = axis_of_rotation(q)
            if sin(q.arg()) == 0
                axis = [0, 0, 0];
            else
                axis = q.imag/sin(q.arg());
            end
        end
            
        % Returns its conjugate
        function conjugate = conj(self)
            conjugate = [self.w, -self.i, -self.j, -self.k];
        end
        
    end
end