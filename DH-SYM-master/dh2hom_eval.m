function He = dh2hom_eval(H, q, L)
% Take Robot Manipulator cell array of symbolic H matrices generated from 
% the DH-SYM library with their numeric parameters q and L, and numerically
% evaluate them
%
%   Outputs:
%		
%		He      (N+1) x 1 cell array of numerically evaluated H matrices
%
%	Inputs:
%
%       H 		(N+1) x 1 cell array of symbolic homogeneous 
%               transformations to each i-th link from the base of the 
%               kinematic chain.
%
%       q       N x 1 numeric vector of joint angles
%
%       L       N x 1 numeric vector of link lengths

% Check that a column vector has been passed in
if  size(q,1) < size(q,2)           || ...
    size(L,1) < size(L,2)           
        
    error('myApp:argChk', 'Parameter vectors must be column vectors.')
    
else

    % Create output cell array
    He = cell(size(H));
    % Number of matrices to evaluate
    k = size(H,1);
    % Number of joint variables
    n = size(q,1);
   
    % Create new variables q1, q2, etc. and set them to q(1), q(2), etc.
    % L1, L2, etc. and set them to l(1), l(2), etc.
    for i = 1:n
        eval([genvarname(['q',num2str(i)]),     '= q(i);']);
        eval([genvarname(['L',num2str(i)]),     '= L(i);']);
    end

    % Numerically evaluate matrices
    for i = 1:k
        He{i} = vpa(subs(H{i}));
    end

end