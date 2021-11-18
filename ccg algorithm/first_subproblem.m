% function theta_x = first_subproblem(nyc,nyd,P,N,R,K,w,v,x)
% %% The first subproblem in Step 4
% ly = zeros(nyc+nyd,1);
% Ayineq = [P,N];
% byineq = R-K*x;
% cy = [w;v];
% ctypesy = [repmat('C',nyc,1);repmat('B',nyd,1)];
% [~,theta_x,~]=cplexmilp(-cy,Ayineq,byineq,[],[],[],[],[],ly,[],ctypesy');
% theta_x = -theta_x;
% end

function theta_x = first_subproblem(model,A0,nyc,nyd,P,N,R,K,w,v,x)
fsubmodel.A=[P,N];
fsubmodel.rhs=[R-K*x];
fsubmodel.obj=[w;v];
fsubmodel.modelsense = 'Max';
fsubmodel.sense=model.sense(size(A0,1)+1:size(A0,1)+size(P,1)); 
fsubmodel.vtype=[repmat('C',nyc,1);repmat('I',nyd,1)];
ssubmodel.lb=model.lb(size(K,2)+1:size(K,2)+nyc+nyd);
ssubmodel.ub=model.ub(size(K,2)+1:size(K,2)+nyc+nyd);
r=gurobi(fsubmodel);
theta_x=r.objval;
end

