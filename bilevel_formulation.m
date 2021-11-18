%% problem formulation 
%%%   The problem is a bilevel model           
%%%   Upper Level -detailed BSS management; x includes [swaprice, char-rate, load, soc_new, FB_stock...];
%%%   Lower level -amod operation; y includes [reb, preswawp, N_final]; z includes [swap, queue];

%%% 
% function model = bilevel_formulation()
% 参数放到一起方便调参
clc;clear;
% road netwoek settings 
T=24;
C=10;
N=11;
roadgraph{1}=[2,6,9,11];
roadgraph{2}=[1,7];
roadgraph{3}=[4,5,6];
roadgraph{4}=[3,7];
roadgraph{5}=[3,6,8,9];
roadgraph{6}=[1,3,5,7,9];
roadgraph{7}=[2,4,6];
roadgraph{8}=[5,9,10];
roadgraph{9}=[1,5,6,8,10];
roadgraph{10}=[8,9,11];
roadgraph{11}=[1,10];
tvroadcap=zeros(T,N,N);
for t=1:T
    tvroadcap(t,:,:)=1000;
end

traveldist=zeros(N);
for i=1:N
    for j=roadgraph{i}
        if j~=i
            traveldist(i,j)=1;
        else
            traveldist(i,j)=0;
        end
    end
end

chargetotraverse=traveldist;

traveltime=zeros(N);
for i=1:N
    for j=roadgraph{i}
        traveltime(i,j)=1;
    end
end
[routetime,routecharge,routes] = build_routes(roadgraph,traveltime,chargetotraverse);

% BSS settings
N_bay=20;
N_initial=100; %Ful battery的初始数目
soc_initial=5; %电池的初始SOC
swaplist=[3,6,9];
W=length(swaplist);

for w=1:W
    swaptime(w)=1;
end  %假定所有的BSS一样

% swapcharge=[3,3,3,3];
swapcap=zeros(T,W);
cap_queue=zeros(T,W);

for t=1:T
    for w=1:W
        swapcap(t,w)=40;
        cap_queue(t,w)=10;
    end
end

bigM=13; %BSS detailed charging management的约束线性化时候用到的
% initial position of vehicles
Empty_InitialPosRT=zeros(T,N,C);
MinEndCharge=6;
%起始共有300辆车 每个节点30
for t=1
    for i=1:N
        for c=6
            Empty_InitialPosRT(t,i,c)=30;
        end
    end
end

%改变初始车辆数目-2000 改变换电站-100/道路容量-inf 改变换电价格 对换电结果都会产生影响

%换电站收益和
sum_profit=zeros(T);
for t=1:T
    sum_profit(t)=100;
end

matrix_A=eye(T*C*W);
matrix_b=ones(T*C*W,1);

% 请求有
sources=load('sources.txt');
sinks=load('sinks.txt');
startimes=load('startimes.txt');
flows=ceil(1.5*load('flows.txt'));

%目标函数里各部分的系数
%上层目标函数的系数
coeffi_queue=100;  %排队成本

p=[ 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 7 7 7 7 7 7 7 7 5 5 5 5 5 5 5 5 ... 
    0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 7 7 7 7 7 7 7 7 5 5 5 5 5 5 5 5 ...
    0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 7 7 7 7 7 7 7 7 5 5 5 5 5 5 5 5 ]; %TOU电价
pi=zeros(T,W);
for t=1:T
    for w=1:W
        pi(t,w)=p((w-1)*T+t);
    end
end

%下层目标函数的系数
coeffi_cus=50;  %接单收益 应该是跟订单的距离成比例，不应该是一个常数
coeffi_reb=2;      %再调度车辆的成本
coeffi_sourcerelax=200; %没按时满足订单的惩罚

%对需求进行bundle
csinks=unique(sinks);
csources=cell(size(csinks));
cstartimes=cell(size(csinks));
cflows=cell(size(csinks));

M=length(csinks);
k=0;
for  i=1:M
    k=(sinks==csinks(i)); %数组的逻辑索引 巧妙 速度快
    csources{i}=sources(k);
    cstartimes{i}=startimes(k);
    cflows{i}=flows(k);
end

sources=csources;
sinks=csinks';
startimes=cstartimes';
flows=cflows';

%outstanding customers demand
startsources=1;
startsinks=3;
startflows=50;

cstartsinks=      unique(startsinks);
cstartsources=    cell(size(cstartsinks));
cstartflows=      cell(size(cstartsinks));

MS=length(cstartsinks);
for i=1:MS
    cstartsources{i}=startsources(startsinks==cstartsinks(i));
    cstartflows{i}=startflows(startsinks==cstartsinks(i));
end

startsources=cstartsources;
startsinks=cstartsinks';
startflows=cstartflows';

for k=1:M
    s(k)=length(sources{k});    
end
S=max(s); 
for k1=1:MS
    ss(k1)=length(startsources{k1});
end
S_S=max(ss);

%%% upper level constraints (BSS  operation model)

% decision variables %
%%%%     upper level variables ('x' in the paper)
%%%%     SWAPRICE, DB_STOCK, FB_STOCK, LOAD, UNLOAD, SOC_NEW, SOC, CHAR_RATE,
%%%%     explanations of each dimension: t-time, b-charging bay,
%%%%     w-swapping station, c-state of charge of battery

for t=1:T
    for c=1:C
        for w=1:W
            swaprice(t,c,w)=(t-1)*C*W+(c-1)*W+w;
        end
    end
end

for t=1:T
    for c=1:C
        for w=1:W
            S_dep(t,c,w)=T*C*W+(t-1)*C*W+(c-1)*W+w;
        end
    end
end

for t=1:T
    for w=1:W
       S_full(t,w)=2*T*C*W+(t-1)*W+w;
    end
end

for t=1:T
    for b=1:N_bay
        for c=1:C
            for w=1:W
                nload(t,b,c,w)=2*T*C*W+T*W+(t-1)*N_bay*C*W+(b-1)*C*W+(c-1)*W+w;
            end
        end
    end
end

for t=1:T
    for b=1:N_bay
        for w=1:W
            unload(t,b,w)=2*T*C*W+T*W+T*N_bay*C*W+(t-1)*N_bay*W+(b-1)*W+w;
        end
    end
end

for t=1:T
    for b=1:N_bay
        for w=1:W
            soc_new(t,b,w)=2*T*C*W+T*W+T*N_bay*C*W+T*N_bay*W+(t-1)*N_bay*W+(b-1)*W+w;
        end
    end
end

for t=1:T
    for b=1:N_bay
        for w=1:W
            soc(t,b,w)=2*T*C*W+T*W+T*N_bay*C*W+2*T*N_bay*W+(t-1)*N_bay*W+(b-1)*W+w;
        end
    end
end

for t=1:T
    for b=1:N_bay
        for w=1:W
            char_rate(t,b,w)=2*T*C*W+T*W+T*N_bay*C*W+3*T*N_bay*W+(t-1)*N_bay*W+(b-1)*W+w;
        end
    end
end

for t=1:T
    for b=1:N_bay
        for w=1:W
            auxi_y(t,b,w)=2*T*C*W+T*W+T*N_bay*C*W+4*T*N_bay*W+(t-1)*N_bay*W+(b-1)*W+w;
        end
    end
end

for t=1:T
    for b=1:N_bay
        for w=1:W
            auxi_z(t,b,w)=2*T*C*W+T*W+T*N_bay*C*W+5*T*N_bay*W+(t-1)*N_bay*W+(b-1)*W+w;
        end
    end
end

totalx=2*T*C*W+T*W+T*N_bay*C*W+6*T*N_bay*W;

%%% lower level decision variables %%%
% continuous variables(y)--cus_in(c,k,sour),cus_out(t,c,k),waitcus_out,waitcus_in,
% continuous variables(y)--reb(t,c,i,j), preswap(t,c,w), find_end(c,i)
% discrete veriables(z)--swap(t,c,w), queue(t,c,w)
%s--swapping station n---node in transportation network

for c=1:C
    for k=1:M
        for sour=1:S
            cus_in(c,k,sour)=totalx+sour+(k-1)*S+(c-1)*M*S;
        end
    end
end
for t=1:T
    for c=1:C
        for k=1:M
            cus_out(t,c,k)=totalx+C*M*S+k+(c-1)*M+(t-1)*C*M;
        end
        for k1=1:MS
            waitcus_out(t,c,k1)=totalx+C*M*S+T*C*M+k1+(c-1)*MS+(t-1)*C*MS;
            for sso=1:S_S
                waitcus_in(t,c,k1,sso)=totalx+C*M*S+T*C*M+T*C*MS+sso+(k1-1)*S_S+(c-1)*MS*S_S+(t-1)*C*MS*S_S;
            end
        end
        for i=1:N
            find_end(c,i)=totalx+C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+i+(c-1)*N;
            for j=1:N
                road_reb(t,c,i,j)=totalx+C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+j+(i-1)*N+(c-1)*N*N+(t-1)*C*N*N;
            end
        end
        for w=1:W
            preswap(t,c,w)=totalx+C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+T*C*N*N+(t-1)*C*W+(c-1)*W+w;
        end
    end
end

for k=1:M
    for sour=1:S
        source_rel(k,sour)=totalx+C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+T*C*N*N+T*C*W+sour+(k-1)*S;
    end
end

for k1=1:MS
    for sso=1:S_S
        start_rel(k1,sso)=totalx+C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+T*C*N*N+T*C*W+M*S+sso+(k1-1)*S_S;
    end
end
totaly=C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+T*C*N*N+T*C*W+M*S+MS*S_S;
for t=1:T
    for c=1:C
        for w=1:W
            swap(t,c,w)=totalx+totaly+(t-1)*C*W+(c-1)*W+w;
            queue(t,c,w)=totalx+totaly+T*C*W+(t-1)*C*W+(c-1)*W+w;
        end
    end
end
totalz=2*T*C*W;

total=totalx+totaly+totalz;
model.A=sparse(100,total);
model.rhs = zeros(100,1);
model.sense = zeros(0,1);
row=1;

% 初始条件 %
%%% 初始时刻DB数目为0
for w=1:W
    for t=1:T
        if t==1
            for c=1:C
                model.A(row,S_dep(t,c,w))=1;
                model.rhs(row)=0;
                model.sense=[model.sense,'='];
                row=row+1;
            end
        end
    end
end

for w=1:W
    for t=1:T
        if  t==1
            model.A(row,S_full(t,w))=1;
            model.rhs(row)=N_initial;
            model.sense=[model.sense,'='];
            row=row+1;
        end
    end
end

%initial SoC of battery is the same
for w=1:W
    for t=1:T
        if t==1
            for b=1:N_bay
                model.A(row,soc(t,b,w))=1;
                model.rhs(row)=soc_initial;
                model.sense=[model.sense,'='];
                row=row+1;
            end
        end
    end
end
%unidrnd(5):1-5的随机数
% a=randperm(,1)
                
%%% s.t. %%%
%t时刻 soc=c的DB的库存数目
for w=1:W
    for t=1:T-1
        for c=1:C
            for b=1:N_bay
                model.A(row,nload(t+1,b,c,w))=1;
            end
            model.A(row,swap(t,c,w))=-1;
            model.A(row,S_dep(t+1,c,w))=1;
            model.A(row,S_dep(t,c,w))=-1;
            model.rhs(row)=0;
            model.sense=[model.sense,'='];
            row=row+1;
        end
    end
end

%满电量电池的库存
for w=1:W
    for t=1:T-1
        for b=1:N_bay
            model.A(row,unload(t+1,b,w))=-1;
        end
        for c=1:C
            model.A(row,swap(t,c,w))=1;
        end
        model.A(row,S_full(t,w))=-1;
        model.A(row,S_full(t+1,w))=1;
        model.rhs(row)=0;
        model.sense=[model.sense,'='];
        row=row+1;
    end
end

%满电量电池的库存大于换电需求
for w=1:W
    for t=1:T
        for c=1:C
            model.A(row,swap(t,c,w))=-1;
        end
        model.A(row,S_full(t,w))=1;
        model.rhs(row)=0;
        model.sense=[model.sense,'>'];
        row=row+1;
    end
end

%Σload(t,b,c)=unload(t,b)
for w=1:W
    for t=1:T
        for b=1:N_bay
            for c=1:C
                model.A(row,nload(t,b,c,w))=1;
            end
            model.A(row,unload(t,b,w))=-1;
            model.rhs(row)=0;
            model.sense=[model.sense,'='];
            row=row+1;
        end
    end
end

%C*unload<soc
for w=1:W
    for t=1:T
        for b=1:N_bay
            model.A(row,unload(t,b,w))=C;
            model.A(row,soc(t,b,w))=-1;
            model.rhs(row)=0;
            model.sense=[model.sense,'<'];
            row=row+1;
        end
    end
end

%if nload(t,b,c)==1 then soc_new(t,b)=c的ILP化
for w=1:W
    for t=1:T
        for b=1:N_bay
            for c=1:C
                model.A(row,nload(t,b,c,w))=c;
            end
            model.A(row,soc_new(t,b,w))=-1;
            model.rhs(row)=0;
            model.sense=[model.sense,'='];
            row=row+1;
        end
    end
end

%soc(t,b)的变化的线性化
for w=1:W
    for t=1:T-1
        for b=1:N_bay
            model.A(row,soc(t+1,b,w))=1;
            model.A(row,soc(t,b,w))=-1;
            model.A(row,char_rate(t,b,w))=-1;
            model.A(row,auxi_y(t,b,w))=-1;
            model.A(row,auxi_z(t,b,w))=1;
            model.rhs(row)=0;
            model.sense=[model.sense,'='];
            row=row+1;
        end
    end
end

%linearization of y(t,b)
for w=1:W
    for t=1:T
        for b=1:N_bay
            model.A(row,soc_new(t,b,w))=1;
            model.A(row,unload(t,b,w))=C-1;
            model.A(row,auxi_y(t,b,w))=-1;
            model.rhs(row)=C-1;
            model.sense=[model.sense,'<'];
            row=row+1;
        end
    end
end

for w=1:W
    for t=1:T
        for b=1:N_bay
            model.A(row,unload(t,b,w))=-(C-1);
            model.A(row,auxi_y(t,b,w))=1;
            model.rhs(row)=0;
            model.sense=[model.sense,'<'];
            row=row+1;
        end
    end
end

for w=1:W
    for t=1:T
        for b=1:N_bay
            model.A(row,soc_new(t,b,w))=-1;
            model.A(row,auxi_y(t,b,w))=1;
            model.rhs(row)=0;
            model.sense=[model.sense,'<'];
            row=row+1;
        end
    end
end

%linearization of z(t,b)
%M 考虑到char_rate的soc的上限
for w=1:W
    for t=1:T
        for b=1:N_bay
            model.A(row,soc(t,b,w))=1;
            model.A(row,unload(t,b,w))=bigM;
            model.A(row,auxi_z(t,b,w))=-1;
            model.rhs(row)=bigM;
            model.sense=[model.sense,'<'];
            row=row+1;
        end
    end
end

for w=1:W
    for t=1:T
        for b=1:N_bay
            model.A(row,unload(t,b,w))=-bigM;
            model.A(row,auxi_z(t,b,w))=1;
            model.rhs(row)=0;
            model.sense=[model.sense,'<'];
            row=row+1;
        end
    end
end

for w=1:W
    for t=1:T
        for b=1:N_bay
            model.A(row,soc(t,b,w))=-1;
            model.A(row,auxi_z(t,b,w))=1;
            model.rhs(row)=0;
            model.sense=[model.sense,'<'];
            row=row+1;
        end
    end
end

%换电站的收入和是CONST 对于x_transpose * y =CONST 的reformulation linearization
% model.A(row:row+T*C*W-1,1:T*C*W)=matrix_A;
% model.rhs(row:row+T*C*W-1)=matrix_b;
% model.sense=[model.sense,repmat('=',1,size(matrix_b,1))];
% 
% model.A(row+T*C*W,totalx+totaly:totalx+totaly+T*C*W-1)=matrix_b'*inv(matrix_A');
% model.rhs(row+T*C*W)=sum_profit;
% model.sense=[model.sense,repmat('=',1,1)];

%上层问题系数矩阵的行数
% rowa=row+T*C*W;
% 
% row=row+T*C*W+1;

%MCcormick relaxation

for t=1:T
    for w=1:W
        for c=1:C
            model.A(row,multi(t,c,w))=1;
        end
    end
    model.rhs(row)=sum_profit(t);
    model.sense=[model.sense,'='];
    row=row+1;
end


for t=1:T
    for w=1:W
        for c=1:C
            model.A(row,multi(t,c,w))=1;
            model.A(row,swaprice(t,c,w))=-num_lb;
            model.A(row,swap(t,c,w))=-price_lb;
            model.rhs(row)=-num_lb*price_lb;
            model.sense=[model.sense,'>'];
            row=row+1;
        end
    end
end

for t=1:T
    for w=1:W
        for c=1:C
            model.A(row,multi(t,c,w))=1;
            model.A(row,swaprice(t,c,w))=-num_ub;
            model.A(row,swap(t,c,w))=-price_ub;
            model.rhs(row)=-num_ub*price_ub;
            model.sense=[model.sense,'>'];
            row=row+1;
        end
    end
end

for t=1:T
    for w=1:W
        for c=1:C
            model.A(row,multi(t,c,w))=1;
            model.A(row,swaprice(t,c,w))=-num_lb;
            model.A(row,swap(t,c,w))=-price_ub;
            model.rhs(row)=-num_lb*price_ub;
            model.sense=[model.sense,'<'];
            row=row+1;
        end
    end
end

for t=1:T
    for w=1:W
        for c=1:C
            model.A(row,multi(t,c,w))=1;
            model.A(row,swaprice(t,c,w))=-num_ub;
            model.A(row,swap(t,c,w))=-price_lb;
            model.rhs(row)=-num_ub*price_lb;
            model.sense=[model.sense,'<'];
            row=row+1;
        end
    end
end
rowa=row;
%%% 目标函数 min cost-battery charging+cost-queue

model.obj=zeros(total,1);
model.obj(1:totalx+totaly+totalz)=0; 
model.modelsense = 'max';

for t=1:T
    for b=1:N_bay
        for w=1:W
            model.obj(char_rate(t,b,w))=-pi(t,w);
        end
    end
end
       
%pi=[0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 7 7 7 7 7 7 7 7 5 5 5 5 5 5 5 5 ];
%pi=[5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5];
for t=1:T
    for c=1:C
        for w=1:W
          model.obj(queue(t,c,w))=-coeffi_queue;
        end
    end
end

%% lower level formulation (AMOD operation model)

%%% s.t.
%%% Flags
sourcerelaxflag=1; 

%%% flow conservation
for t=1:T
    for c=1:C
        for i=1:N
            if ~isempty(roadgraph{i})
                for j=roadgraph{i} %Out-flows
                    if ((chargetotraverse(i,j)<c) && (t+full(traveltime(i,j))<=T))
                       model.A(row,road_reb(t,c,i,j))=1;
                    end
                end
            end
            if ~isempty(roadgraph{i}) % ~isempty(ReverseRoadGraph{i})
                for j=roadgraph{i} %In-flows
                    if (chargetotraverse(j,i)+c<=C && t-full(traveltime(j,i))>0)
                        model.A(row,road_reb(t-full(traveltime(j,i)),chargetotraverse(j,i)+c,j,i))=-1;  
                    end
                end
            end
            for k=1:M
                for sour=1:length(sources{k})
                    if (sources{k}(sour)==i && startimes{k}(sour)==t)
                        model.A(row,cus_in(c,k,sour))=1;
                        % Departing passengers (exiting vehicles) at charge level c
                    end
                end
                if (sinks(k)==i)
                    model.A(row,cus_out(t,c,k))=-1;                    
                    %Arriving passengers (entering vehicles)
                end
            end
            for w=1:W
                if (swaplist(w)==i) % preswap(t,c,w)
                    if (t<=T)&&(c<C)
                        model.A(row,preswap(t,c,w))=1;
                    end
                    if t>swaptime(w)&&(c==C) %swap(t-deltat,c,w)
                        for cc=1:C-1
                            model.A(row,swap(t-swaptime(w),cc,w))=-1;
                        end
                    end
                end
            end
            if t==T
                model.A(row,find_end(c,i))=1;
            end
            model.rhs(row)=Empty_InitialPosRT(t,i,c);
            model.sense=[model.sense,'='];
            row=row+1;
        end
    end
end

% queue conservation    queue+preswap=swap+queue
for t=2:T
    for c=1:C
        for w=1:W
            model.A(row,queue(t-1,c,w))=1;
            model.A(row,preswap(t,c,w))=1;
            model.A(row,swap(t,c,w))=-1;
            model.A(row,queue(t,c,w))=-1;
            model.rhs(row)=0;
            model.sense=[model.sense,'='];
            row=row+1;
        end
    end            
end

%Sum of all FindPaxSourceChargeck = Pax. source
for k=1:M
    for sour=1:length(sources{k})
        for c=1:C
            model.A(row,cus_in(c,k,sour))=1;
        end
        if sourcerelaxflag
            model.A(row,source_rel(k,sour))=1;
        end
        model.rhs(row)=flows{k}(sour);
        model.sense=[model.sense,'='];
        row=row+1;
    end
end

% Sum of all FindPaxSinkChargeck = Pax. sink
for k=1:M
    for c=1:C
        for t=1:T
           model.A(row,cus_out(t,c,k))=1;
        end
    end
    if sourcerelaxflag
        for sour=1:length(sources{k})
            model.A(row,source_rel(k,sour))=1;
        end
    end    
    model.rhs(row)=sum(flows{k});
    model.sense=[model.sense,'='];
    row=row+1;
end

% Conservation of customer charge
for k=1:M
    for t=1:T
        for c=1:C
            model.A(row,cus_out(t,c,k))=-1;
            for sour=1:length(sources{k})
                if startimes{k}(sour)==t-routetime(sources{k}(sour),sinks(k))
                    if c+routecharge(sources{k}(sour),sinks(k))<=C && c+routecharge(sources{k}(sour),sinks(k))>0
                       model.A(row,cus_in(c+routecharge(sources{k}(sour),sinks(k)),k,sour))=1;
                    end
                end
            end
            model.rhs(row)=0;    
            model.sense=[model.sense,'='];
            row=row+1;
        end
    end
end

% 车辆和的约束（因为流守恒的约束，所以这个约束应该不是必要的）
for t=1:T
    for c=1:C
        for i=1:N
            for j=roadgraph{i}
                model.A(row,road_reb(t,c,i,j))=1;
            end
        end
    end
    for c=1:C
        for w=1:W
            model.A(row,preswap(t,c,w))=1;
        end
   end
   model.rhs(row)=sum(sum(sum(Empty_InitialPosRT)));   
   model.sense=[model.sense,'='];
   row=row+1;
end

%%%  INEQUALITY CONSTRAINTS
% queue:congestion  
for t=1:T
    for w=1:W
        for c=1:C
           model.A(row,queue(t,c,w))=1;          
        end
        model.rhs(row)=cap_queue(t,w);
        model.sense=[model.sense,'<'];
        row=row+1;      
    end
end

% Roads: congestion
for t=1:T
    for i=1:N
        for j=roadgraph{i}
            for c=1:C
                model.A(row,road_reb(t,c,i,j))=1;
            end                     
            model.rhs(row)=tvroadcap(t,i,j);
            model.sense=[model.sense,'<'];
            row=row+1;
        end
    end
end

%这个约束是要表达啥？？？？？ 
% for t=T
%     for i=1:N
%         for c=1:C
%             model.A(row,find_end(c,i))=c;
%         end
%     end
%     model.rhs(row)=600;
%     model.sense=[model.sense,'>'];
%     row=row+1;
% end

%%% Upper and lower bounds

model.lb=zeros(total,1);
model.ub=Inf*ones(total,1);

% Don't create chargers/dischargers from thin air
% for n=1:SS
%     for t=T-swaptime(n)+1:T
%         for c=1:C
%             model.ub(swap_reb(t,c,n))=0;
%         end
%     end
% end

% Number of relaxed pax should never be negative
if sourcerelaxflag
    for k=1:M
        for sour=1:length(sources{k})
            model.ub(source_rel(k,sour))=flows{k}(sour);
            model.lb(source_rel(k,sour))=0;
        end
    end
end

%sigma_c swap(t,c,w)<FB_stock(t,w)
for t=1:T
    for w=1:W
        for c=1:C
            model.A(row,swap(t,c,w))=1;
        end
        model.A(row,S_full(t,w))=-1;
        model.rhs(row)=0;
        model.sense=[model.sense,'<'];
        row=row+1;
    end
end

%%% 目标函数 max profit_cus-cost_reb-cost_swap
model.obj=zeros(total,1);
model.modelsense = 'Max';
for c=1:C
    for k=1:M
        for sour=1:length(sources{k})
            model.obj(cus_in(c,k,sour))=coeffi_cus;
        end
    end
end

for t=1:T
    for c=1:C
        for i=1:N
            for j=roadgraph{i}
                model.obj(road_reb(t,c,i,j))=-coeffi_reb;
            end
        end
    end
end

if sourcerelaxflag 
for k=1:M
    for sour=1:length(sources{k})
        model.obj(source_rel(k,sour))=-coeffi_sourcerelax;
    end
end
end     

%% UB&LB

for t=1:T
    for b=1:N_bay
        model.ub(soc(t,b))=C;
    end
end

for t=1:T
    for b=1:N_bay
        for w=1:W
            model.ub(auxi_y(t,b))=C-1;
            model.ub(auxi_z(t,b))=bigM;
            model.lb(soc_new(t,b))=0;
        end
    end
end

for t=1:T
    for b=1:N_bay
       model.ub(char_rate(t,b))=6;
    end
end

model.lb=zeros(total,1);
model.ub=Inf*ones(total,1);

% Don't create chargers/dischargers from thin air
% for n=1:SS
%     for t=T-swaptime(n)+1:T
%         for c=1:C
%             model.ub(swap_reb(t,c,n))=0;
%         end
%     end
% end

% Number of relaxed pax should never be negative
if sourcerelaxflag
    for k=1:M
        for sour=1:length(sources{k})
            model.ub(source_rel(k,sour))=flows{k}(sour);
            model.lb(source_rel(k,sour))=0;
        end
    end
end

% Vehicles: Minimum end charge
% for i=1:N
%     for c=1:MinEndCharge
%         model.ub(find_end(c,i))=0;
%     end
% end
%% Define variables types and standard compact model
model.vtype(1:T*C*W)='C';  %swaprice
model.vtype(T*C*W+1:totalx)='I'; %x变量里的其他变量
model.vtype(totalx+1:totalx+totaly)='C'; %y variables
model.vtype(totalx+totaly+1:totalx+totaly+totalz)='I'; %z varaibles

model.f=[zeros(2*T*C*W+T*W+T*N_bay*C*W+3*T*N_bay*W,1);...
    model.obj(2*T*C*W+T*W+T*N_bay*C*W+3*T*N_bay*W+1:2*T*C*W+T*W+T*N_bay*C*W+4*T*N_bay*W);...
    zeros(totalx-(2*T*C*W+T*W+T*N_bay*C*W+4*T*N_bay*W),1)];
model.g=zeros(totaly,1);
model.h=[zeros(T*C*W,1);...
    model.obj(totalx+totaly+T*C*W+1:totalx+totaly+totalz)];
model.w=zeros(totaly,1); %先开辟空间出来
model.w(1:C*M*S)=model.obj(totalx+1:totalx+C*M*S); %cus in
model.w(C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+1:C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+T*C*N*N)=...
    model.obj(totalx+C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+1:totalx+C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+T*C*N*N); %road_reb
model.w(C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+T*C*N*N+T*C*W+1:C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+T*C*N*N+T*C*W+M*S)=...
    model.obj(totalx+C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+T*C*N*N+T*C*W+1:totalx+C*M*S+T*C*M+T*C*MS+T*C*MS*S_S+C*N+T*C*N*N+T*C*W+M*S);%source-relax
model.v=[model.obj(totalx+totaly+1:totalx+totaly+T*C*W);zeros(totalz-(T*C*W),1)]; %swap  
model.b=model.rhs(1:rowa);
model.A0=model.A(1:rowa,1:totalx);
model.K=model.A(rowa+1:row-1,1:totalx);
model.P=model.A(rowa+1:row-1,totalx+1:totalx+totaly);
model.N=model.A(rowa+1:row-1,totalx+totaly+1:totalx+totaly+totalz);
model.R=model.rhs(rowa+1:row-1);

%gurobi_write(model,'qp.lp');
%result=gurobi_iis(model);
result=gurobi(model);














