classdef quaternion_utils
    % QUATERNION_UTILS Class containing utility functions for quadrotor estimation
    % This class provides static methods for rotation matrices, coordinate transformations
    % and other mathematical operations required for quadrotor dynamics
    
    methods(Static)
        function Rb = RbMatrix(phi, theta, psi)
            
            % RBMATRIX Compute the rotation matrix from world to body frame
            Rb = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
                  cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
                  -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
        end
        
        function [Q, Qdot] = EulerToBodyAngularVelocityMatrix(phi, theta, phi_d, theta_d)
            
            % EULERTOBODYANGULARVELOCITYMATRIX Compute the transformation matrix
            % from Euler angle rates to body angular velocities and its time derivative
             Q = [1, 0, -sin(theta);
                 0, cos(phi), cos(theta)*sin(phi);
                 0, -sin(phi), cos(theta)*cos(phi)];
             
            Qdot = [0, 0, -cos(theta)*theta_d;
                    0, -sin(phi)*phi_d, -sin(theta)*theta_d*sin(phi) + cos(theta)*cos(phi)*phi_d;
                    0, -cos(phi)*phi_d, -sin(theta)*theta_d*cos(phi) - cos(theta)*sin(phi)*phi_d];
        end
        
        function S = skew(w)
           
               % SKEW Convert a 3D vector to a skew-symmetric matrix
               S = [0, -w(3), w(2);
                     w(3), 0, -w(1);
                     -w(2), w(1), 0];
        end
        
        function C = CMatrix(eta_dot, Q, Qdot, I_wrtb)
            
            % CMATRIX Compute the Coriolis matrix for quadrotor dynamics
            S = quaternion_utils.skew(Q*eta_dot');
            C = Q'*S*I_wrtb*Q + Q'*I_wrtb*Qdot;
        end 
        
         function K = computeKCoefficients(r, c)
            
            K = zeros(r, 1);
            acc = 1;
            for j = 1:r
                K(j) = c(j)/acc;
                acc = acc*K(j);
            end
            K = flip(K);
        end
    end
end