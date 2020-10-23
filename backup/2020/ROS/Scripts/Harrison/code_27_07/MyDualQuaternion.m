classdef MyDualQuaternion
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        real % The real quaternion (first quaternion)
        dual % The dual quaternion (dual number)
    end
    
    methods
        % Construction of the dual-quaternion
        function obj = MyDualQuaternion(first_quaternion, second_quaternion)
        obj.real = first_quaternion;
        obj.dual = second_quaternion;
        end
        
        % Returns the dual-quaternion constructed
        function sigma = dual_quaternion(self)
            sigma = [self.real, self.dual];
        end
        
        % Returns its real quaternion
        function real_quaternion = real_part(self)
           real_quaternion = self.real;
        end
        
        % Returns its dual quaternion
        function dual_quaternion = dual_part(self)
           dual_quaternion = self.dual;
        end
        
        % Returns its first conjugate (the dual number is conjugated)
        function result_1 = conjugate_1(self)
           result_1 = [self.real(), -self.dual()];
        end
        
        % Returns its second conjugate (both quaternions are conjugated
        % individually)
        function result_2 = conjugate_2(self)
           r = [self.real(1), -self.real(2), -self.real(3), -self.real(4)];
           t = [self.dual(1), -self.dual(2), -self.dual(3), -self.dual(4)];
           result_2 = [r, t];
        end
        
        % Returns its third conjugate (the two previous conjugates are
        % applied simultaneously)
        function result_3 = conjugate_3(self)
           r = [self.real(1), -self.real(2), -self.real(3), -self.real(4)];
           t = [-self.dual(1), self.dual(2), self.dual(3), self.dual(4)];
           result_3 = [r, t];
        end
        
        % Returns its measurement of magnitude:
        function result = magnitude(self)
           aux_conjugate_2 = self.conjugate_2();
           rot = quat_mul(self.real(), aux_conjugate_2(1:4));
           trans = quat_mul(self.real(), aux_conjugate_2(5:8)) + quat_mul(self.dual(), aux_conjugate_2(1:4));
           result = [rot, trans];
        end
        
        
    end
end

