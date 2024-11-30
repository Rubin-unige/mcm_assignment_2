%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType)
            if nargin > 1
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end

        function updateDirectGeometry(self, q)
            %% GetDirectGeometryFunction
            % This method updates the matrices iTj for the given joint configuration q.
            % Inputs:
            % q : joints current position (in radians for rotational joints, meters for prismatic joints)
            % The function updates:
            % - iTj: vector of matrices containing the transformation matrices from link i to link j for the input q.
            % The size of iTj is equal to (4,4,numberOfLinks)
        
            % Iterate through each joint
            for i = 1:self.jointNumber
                % Extract the current transformation matrix for this joint from iTj_0
                iTj_current = self.iTj_0(:,:,i); 

                if self.jointType(i) == 0  % If the joint is rotational  
                    R_zero_state = iTj_current(1:3, 1:3);  % extract the rotational matrix from transformation
        
                    % Create new rotation matrix for rotaion around z
                    theta = q(i);
                    % rotation around z
                    R_z = [cos(theta), -sin(theta), 0;
                           sin(theta), cos(theta), 0;
                           0, 0, 1];
        
                    % Multiply the zero state rotaion matrix extracted earlier
                    R_actual = R_zero_state * R_z;
        
                    % Update the transformation matrix with the new rotation
                    iTj_current(1:3, 1:3) = R_actual;
        
                elseif self.jointType(i) == 1  % If the joint is prismatic
                    % Extract the translation vector from iTj_0, rotation is identity
                    r_zero_state = iTj_current(1:3, 4);  % 3x1 translation vector
        
                    % For a prismatic joint, the displacement is along the z-axis (can be different based on axis)
                    d_trans = q(i);  
        
                    % Update the translation for the prismatic joint
                    r_updated = r_zero_state + [0; 0; d_trans];  
        
                    % Update the transformation matrix with the new translation
                    iTj_current(1:3, 4) = r_updated;
                end
                
                % Update the iTj matrix for this joint
                self.iTj(:,:,i) = iTj_current;
            end
        end

        function [bTk] = getTransformWrtBase(self, k)
            %% GetTransformWrtBase function
            % Inputs:
            % k: the index for which to compute the transformation matrix (frame k)
            % Outputs:
            % bTk: transformation matrix from the manipulator base to the k-th joint (or frame k)
        
            % Start with the identity matrix for the base frame (Frame 0)
            bTk = eye(4);
        
            % Multiply the transformation matrices for joints 1 through k
            for i = 1:k
                % Multiply the current transformation matrix with the transformation matrix for joint i
                bTk = bTk * self.iTj(:,:,i);
            end
        end
    end
end


