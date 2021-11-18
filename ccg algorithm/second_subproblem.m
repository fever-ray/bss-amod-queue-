% function [y,z,fval_yz] = second_subproblem(nyc,nyd,P,N,R,K,w,v,g,h,x,theta_x)
% %% The second subproblem in Step 4
% lyz = zeros(nyc+nyd,1);
% Ayzineq = [P,N];
% byzineq = R-K*x;
% Ayzineq = [Ayzineq;-w',-v'];
% byzineq = [byzineq;-theta_x];
% cyz = [g;h];
% ctypesyz = [repmat('C',nyc,1);repmat('B',nyd,1)];
% [yz,fval_yz,~]=cplexmilp(cyz,Ayzineq,byzineq,[],[],[],[],[],lyz,[],ctypesyz');
% y = yz(1:nyc);
% z = yz(nyc+1:end);
% 
% end

function [y,z,Theta] = second_subproblem(model,A0,nyc,nyd,P,N,R,K,w,v,g,h,x,theta_x)
ssubmodel.obj=[g;h];
ssubmodel.A=sparse([P,N;w',v']);   
ssubmodel.rhs=[R-K*x;theta_x];
ssubmodel.modelsense = 'Min';
ssubmodel.sense=[model.sense(size(A0,1)+1:size(A0,1)+size(P,1)),'>']; 
ssubmodel.vtype=[repmat('C',nyc,1);repmat('I',nyd,1)];
ssubmodel.lb=model.lb(size(K,2)+1:size(K,2)+nyc+nyd);
ssubmodel.ub=model.ub(size(K,2)+1:size(K,2)+nyc+nyd);
gurobi_write(ssubmodel,'SP.lp'); %why this 'gruobi write' does not work
r2=gurobi(ssubmodel);
y=r2.x(1:nyc);
z=r2.x(nyc+1:nyc+nyd);
Theta=r2.objval;
end




