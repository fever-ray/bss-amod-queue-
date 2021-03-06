
function resu = bilevel_ccg(model)
%% A column and constrain generation method for bi-level milp prorammings
dbstop if error;
% 1) Decouple the parameters
f = -model.f;
A0 = model.A0;
b = model.b;
g = -model.g;
h = -model.h;
w = model.w;
v = model.v;
P = model.P;
N = model.N;
K = model.K;
R = model.R;

% A=model.A;
% 2) Data structure
nx = length(f); % number of variables in the leader's decision
nyc = length(g);% number of continuous variables in the followers' decision
nyd = length(h);% number of discrete variables in the followers' decision
npi = size(P,1);% number of dual variables  对偶变量的数目等于约束的个数

% 3) Formulate the high point problem
% 3.1) Formulate the high point solution
% model.obj=[f;g;h];
% model.rhs=[b;R];
% model.sense=[repmat('<',1,size(b,1)+size(R,1))];
params.MIPGap=0.005;
result=gurobi(model,params);

xyz=result.x;
x=xyz(1:nx);

% T=12;C=10;W=3;N=11;N_bay=20;M=;S=;MS=;S_S=;
% for t=1:T
%     for c=1:C
%         for w=1:W
%             swaprice(t,c,w)=xyz((t-1)*C*W+(c-1)*W+w);
%         end
%     end
% end
% 
% for t=1:T
%     for c=1:C
%         for w=1:W
%             S_dep(t,c,w)=xyz(T*C*W+(t-1)*C*W+(c-1)*W+w);
%         end
%     end
% end
% 
% for t=1:T
%     for w=1:W
%        S_full(t,w)=xyz(2*T*C*W+(t-1)*W+w);
%     end
% end
% 
% for t=1:T
%     for b=1:N_bay
%         for c=1:C
%             for w=1:W
%                 nload(t,b,c,w)=xyz(2*T*C*W+T*W+(t-1)*N_bay*C*W+(b-1)*C*W+(c-1)*W+w);
%             end
%         end
%     end
% end
% 
% for t=1:T
%     for b=1:N_bay
%         for w=1:W
%             unload(t,b,w)=xyz(2*T*C*W+T*W+T*N_bay*C*W+(t-1)*N_bay*W+(b-1)*W+w);
%         end
%     end
% end
% 
% for t=1:T
%     for b=1:N_bay
%         for w=1:W
%             soc_new(t,b,w)=xyz(2*T*C*W+T*W+T*N_bay*C*W+T*N_bay*W+(t-1)*N_bay*W+(b-1)*W+w);
%         end
%     end
% end
% 
% for t=1:T
%     for b=1:N_bay
%         for w=1:W
%             soc(t,b,w)=xyz(2*T*C*W+T*W+T*N_bay*C*W+2*T*N_bay*W+(t-1)*N_bay*W+(b-1)*W+w);
%         end
%     end
% end
% 
% for t=1:T
%     for b=1:N_bay
%         for w=1:W
%             char_rate(t,b,w)=xyz(2*T*C*W+T*W+T*N_bay*C*W+3*T*N_bay*W+(t-1)*N_bay*W+(b-1)*W+w);
%         end
%     end
% end
% totalx=3*T*C*W+T*W+T*N_bay*C*W+6*T*N_bay*W;
% for c=1:C
%     for k=1:M
%         for sour=1:S
%             cus_in(c,k,sour)=xyz(totalx+sour+(k-1)*S+(c-1)*M*S);
%         end
%     end
% end
% 
% for t=1:T
%     for c=1:C
%         for k=1:M
%             cus_out(t,c,k)=xyz(totalx+C*M*S+k+(c-1)*M+(t-1)*C*M);
%         end
%         for k1=1:MS
%             waitcus_out(t,c,k1)=xyz(totalx+C*M*S+T*C*M+k1+(c-1)*MS+(t-1)*C*MS);
%             for sso=1:S_S
%                 waitcus_in(t,c,k1,sso)=xyz(totalx+C*M*S+T*C*M+T*C*MS+sso+(k1-1)*S_S+(c-1)*MS*S_S+(t-1)*C*MS*S_S);
%             end
%         end
%         for i=1:N
%             find_end(c,i)=xyz(totalx+C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+i+(c-1)*N);
%             for j=1:N
%                 road_reb(t,c,i,j)=xyz(totalx+C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+j+(i-1)*N+(c-1)*N*N+(t-1)*C*N*N);
%             end
%         end
%         for w=1:W
%             preswap(t,c,w)=xyz(totalx+C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+T*C*N*N+(t-1)*C*W+(c-1)*W+w);
%         end
%     end
% end
% 
% totaly=C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+T*C*N*N+T*C*W+M*S+MS*S_S;
% for t=1:T
%     for c=1:C
%         for w=1:W
%             swap(t,c,w)=xyz(totalx+totaly+(t-1)*C*W+(c-1)*W+w);
%             queue(t,c,w)=xyz(totalx+totaly+T*C*W+(t-1)*C*W+(c-1)*W+w);
%         end
%     end
% end

% 3.2) Solve the follower's decision making problem
% 3.2.1) Solve the fisrt subproblem in Step 4
theta_x = first_subproblem(model,A0,nyc,nyd,P,N,R,K,w,v,x);
% 3.2.2) Solve the second subproblem in Step 4
[y,z,Theta] = second_subproblem(model,A0,nyc,nyd,P,N,R,K,w,v,g,h,x,theta_x);

% 4) Start the iteration
k = 1; % The iteration index
UB = f'*x+Theta;  %这里的也是转置？为什么f要用转置啊？-----f:[n*1]
UB_index=[UB];
LB = -inf;
Lb_index = [LB];

% 4) Construct the master problem
Gap = UB-LB;
Gap_index = [Gap];
nx_add = nyc+npi+nyc+npi;
bgM = 1e6;
disp('ITERATION STARTED');
while Gap>=1e-2
    % 4.1) Column generation, add y_k(nyc), \pi_k(npi), I_k(nyc), I_{\pi}_k(npi) to the master problem
    model.vtype=[model.vtype,repmat('C',1,nyc),repmat('C',1,npi),repmat('B',1,nyc),repmat('B',1,npi)];
    model.lb=[model.lb;zeros(nx_add,1)];
    model.ub=[model.ub;100*ones(nx_add,1)];
    model.obj=[model.obj;zeros(nx_add,1)];
    % 4.2 Expand the constriants
    model.A=sparse([model.A,zeros(size(model.A,1),nx_add)]);  
    % Additional constraints (41)
    A_temp=sparse([zeros(1,nx),w',v',zeros(1,(k-1)*nx_add),-w',zeros(1,npi+nyc+npi)]);
    rhs_temp=[v'*z];
    model.A=[model.A;A_temp];
    model.rhs=[model.rhs;rhs_temp];
    model.sense=[model.sense,repmat('>',1,1)];
    
    % Additional constraints (42), the first item, R - Kx - N*z_{k} -
%     Py_{k} \geq 0
    A_temp=sparse([K,zeros(npi,nyc+nyd+(k-1)*nx_add),P,zeros(npi,npi+nyc+npi)]); %对偶变量数目等于约束数目 所以P的行数为npi
    rhs_temp=[R-N*z];
    model.A=[model.A;A_temp];
    model.rhs=[model.rhs;rhs_temp];
    model.sense=[model.sense,repmat('<',1,npi)];
%     
    A_temp=sparse([zeros(nyc,nx+nyc+nyd+(k-1)*nx_add+nyc),P',zeros(nyc,nyc+npi)]); 
    rhs_temp=[w];
    model.A=[model.A;A_temp];
    model.rhs=[model.rhs;rhs_temp];
    model.sense=[model.sense,repmat('>',1,nyc)];  %w is a [1*nyc] matrix and w'is a [nyc*1] matrix  

    % Additional constriant (43), the fist item, y_{k} \leq  I_{k} * bigM
    %对第一个式子进行线性化后 等价约束1

    A_temp=sparse([zeros(nyc,nx+nyc+nyd+(k-1)*nx_add),eye(nyc),zeros(nyc,npi),-eye(nyc)*bgM,zeros(nyc,npi)]); 
    rhs_temp=[zeros(nyc,1)];
    model.A=[model.A;A_temp];
    model.rhs=[model.rhs;rhs_temp];
    model.sense=[model.sense,repmat('<',1,nyc)];  %w is a [1*nyc] matrix and w'is a [nyc*1] matrix      

%     Aineq_temp = sparse([zeros(nyc, nx + nyc + nyd +(k-1)*nx_add + nyc), -P', eye(nyc)*bgM, zeros(nyc, npi)]);
%     bineq_temp = bigM + w;    
    A_temp=sparse([zeros(nyc, nx + nyc + nyd +(k-1)*nx_add + nyc), P', eye(nyc)*bgM, zeros(nyc, npi)]); 
    rhs_temp=[bgM + w];% w is a [nyc*1] matrix  
    model.A=[model.A;A_temp];
    model.rhs=[model.rhs;rhs_temp];
    model.sense=[model.sense,repmat('<',1,nyc)];  
    
    % Additional constriant (43), the second item, \pi_k \leq I_{\pi}_k * bigM    
    A_temp=sparse([zeros(npi, nx + nyc + nyd +(k-1)*nx_add + nyc),eye(npi),zeros(npi,nyc),-eye(npi)*bgM]); 
    rhs_temp=[zeros(npi,1)];  
    model.A=[model.A;A_temp];
    model.rhs=[model.rhs;rhs_temp];
    model.sense=[model.sense,repmat('<',1,npi)];      
    
%     % Additional constriant (43), the second item, R - Kx - N*z_    {k} - Py_{k} \leq (1-I_{\pi}_k) * bigM
    A_temp=sparse([-K,zeros(npi,nyc+nyd+(k-1)*nx_add),-P,zeros(npi,npi+nyc),eye(npi)*bgM]);
    rhs_temp=[N*z - R + bgM];
    model.A=[model.A;A_temp];
    model.rhs=[model.rhs;rhs_temp];
    model.sense=[model.sense,repmat('<',1,npi)];  
    
    % Solve the master problem to obtain the lower boundary and x
    model.modelsense = 'Min';
    params.MIPGap=0.01;
    gurobi_write(model,'bi_ccg.lp');
    resu=gurobi(model,params);
    if strcmp(resu.status,'INFEASIBLE')
        resu=gurobi_iis(model);
    end 
    Lb_index = [Lb_index;resu.objbound];
    
    Gap = UB-resu.objbound;
    %     disp('The gap for the current iteration is:')
    
    fprintf('The gap for the iteration %d is %d \n',k,Gap);
    k=k+1;
    %     disp(Gap);
    Gap_index = [Gap_index;Gap];
    if Gap <=1e-2
        break
    end
    x = resu.x(1:nx);
    % Solve the slave problem to obtain the upper boundary, y and z
    theta_x = first_subproblem(nyc,nyd,P,N,R,K,w,v,x);
    [y,z,Theta] = second_subproblem(nyc,nyd,P,N,R,K,w,v,g,h,x,theta_x);
    UB = f'*x+Theta;
    UB_index = [UB_index;UB];
end

resu.var = resu.x(1:nx+nyc+nyd);

end