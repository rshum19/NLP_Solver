function check_dyn_cst(soln,OCP)
fprintf('--- Checking constraint violations....\n');
% Unmux solution
t = soln.grid.time;
x = soln.grid.state;
u = soln.grid.control;

defects = @trapezoid;

dt = (t(end)-t(1))/(length(t)-1);
f = OCP.model.dynamics(t,x,u);
ceq_dyn = defects(dt,x,f);
ceq_dyn  = reshape(ceq_dyn,numel(ceq_dyn),1);

% check what constraints are violated
eps = OCP.options.fminOpt.TolCon;
idx = find(abs(ceq_dyn) > eps);

if (length(idx) > 0)
    fprintf('  VIOLATION: %d of %d dynamic points where violated \n',length(idx),length(ceq_dyn));
else
    fprintf('  No constraints where violated!\n');
end

end

