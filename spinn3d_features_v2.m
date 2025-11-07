function feat = spinn3d_features_v2(~, q, dq, q_ref, dq_ref, robot, kin)
% 1×42：q(4), dq(4), q_ref(4), dq_ref(4), Δq(4), ex(3), vx(3),
%       四个范数(4), sign(dq)(4), sin(q)(4), cos(q)(4)
q=q(:)'; dq=dq(:)'; q_ref=q_ref(:)'; dq_ref=dq_ref(:)'; n=numel(q);
Jv = zeros(3,n);
try, J = geometricJacobian(robot, q, 'tool'); Jv = J(4:6,:);catch, end
ex=[0 0 0]; vx=[0 0 0];
try
    [~,Tq]=spinn3d_fkine(kin,q); [~,Tr]=spinn3d_fkine(kin,q_ref);
    ex = (Tr(1:3,4)-Tq(1:3,4)).';
    vx = (Jv*dq(:)).';
catch, end
dq_sgn = sign(dq); dq_sgn(~isfinite(dq_sgn))=0;
feat = double([ q, dq, q_ref, dq_ref, ...
                (q_ref-q), ex, vx, ...
                norm(q_ref-q), norm(dq), norm(ex), norm(vx), ...
                dq_sgn, sin(q), cos(q) ]);
feat = reshape(feat,1,42);
end
