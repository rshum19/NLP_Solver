function [D,C,G] = EoM(KE,PE,q,dq)
%% Form D, C, G, B, and F matrices of
	%%
	%% D(q)ddq+C(q,dq)dq+G(q)=B*tau
	%%
	%% where tau=[tau1; tau2]
	%
    syms g
	D=jacobian(jacobian(KE,dq).',dq);
%     D=simplify(D);
    
	N=max(size(q));
    syms C
	for k=1:N,
		for j=1:N,
			C(k,j)=0*g;
			for i=1:N,
				C(k,j)=C(k,j)+1/2*(diff(D(k,j),q(i))+...
					diff(D(k,i),q(j))-...
					diff(D(i,j),q(k)))*dq(i);
			end
		end
    end
    
    C= C;%simplify(C);

	G= jacobian(PE,q).'   ;  %simplify(jacobian(PE,q).');