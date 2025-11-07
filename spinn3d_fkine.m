function [P, Ttool] = spinn3d_fkine(kin, q)
% Numeric FK for base yaw + 3 pitch chain.
if nargin<1 || isempty(kin), kin = struct(); end
if ~isfield(kin,'L'),      kin.L = [0.24 0.214 0.324]; end
if ~isfield(kin,'base_z'), kin.base_z = 0.03; end
L = kin.L(:).'; base_z = kin.base_z;

q = double(q(:).'); q = [q, zeros(1,4-numel(q))];
Rz = @(a)[cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
Ry = @(a)[cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];

O0=[0 0 0].'; S=[0 0 base_z].';
R1=Rz(q(1)); R2=R1*Ry(q(2)); E1=S+R2*[L(1);0;0];
R3=R2*Ry(q(3)); E2=E1+R3*[L(2);0;0];
R4=R3*Ry(q(4)); EE=E2+R4*[L(3);0;0];

P=[O0.';S.';E1.';E2.';EE.'];
Ttool=eye(4); Ttool(1:3,1:3)=R4; Ttool(1:3,4)=EE;
end
