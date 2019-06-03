clc
clear
close all

%% 二阶系统参数设定
num = 4;
den = [1 1 4];
[a1, b, c, d] = tf2ss(num,den);
x = [0;0];
T = 0.1;

N = 200;
R = [1*ones(1,N/4), 2*ones(1,N/4),1*ones(1,N/4), 2*ones(1,N/4)];

sysc = tf(num,den,'inputdelay',T);
[G,H,C,D] = c2dm(a1, b, c, d, T,'tustin');


%%模糊控制器设计
a = newfis('Simple');

a = addvar(a,'input','e',[-2 2]);
a = addmf(a,'input',1,'NB','trapmf',[-2,-2,-1.5,-1]);
a = addmf(a,'input',1,'NM','trimf',[-1.5,-1,-0.5]);
a = addmf(a,'input',1,'NS','trimf',[-1,-0.5,0]);
a = addmf(a,'input',1,'ZR','trimf',[-0.5,0,0.5]);
a = addmf(a,'input',1,'PS','trimf',[0,0.5,1]);
a = addmf(a,'input',1,'PM','trimf',[0.5,1,1.5]);
a = addmf(a,'input',1,'PB','trapmf',[1,1.5,2,2]);

a = addvar(a,'input','de',[-2 2]);
a = addmf(a,'input',2,'NB','trapmf',[-2,-2,-1.5,-1]);
a = addmf(a,'input',2,'NM','trimf',[-1.5,-1,-0.5]);
a = addmf(a,'input',2,'NS','trimf',[-1,-0.5,0]);
a = addmf(a,'input',2,'ZR','trimf',[-0.5,0,0.5]);
a = addmf(a,'input',2,'PS','trimf',[0,0.5,1]);
a = addmf(a,'input',2,'PM','trimf',[0.5,1,1.5]);
a = addmf(a,'input',2,'PB','trapmf',[1,1.5,2,2]);

a = addvar(a,'output','u',[-2 2]);
a = addmf(a,'output',1,'NB','trapmf',[-2,-2,-1.5,-1]);
a = addmf(a,'output',1,'NM','trimf',[-1.5,-1,-0.5]);
a = addmf(a,'output',1,'NS','trimf',[-1,-0.5,0]);
a = addmf(a,'output',1,'ZR','trimf',[-0.5,0,0.5]);
a = addmf(a,'output',1,'PS','trimf',[0,0.5,1]);
a = addmf(a,'output',1,'PM','trimf',[0.5,1,1.5]);
a = addmf(a,'output',1,'PB','trapmf',[1,1.5,2,2]);

r_pre = [ 7 6 6 5 5 4 4;
          6 6 5 5 4 4 3;
          6 5 5 4 4 3 3;
          5 5 4 4 3 3 2;
          5 4 4 3 3 2 2;
          4 4 3 3 2 2 1;
          4 3 3 2 2 1 1]; 

rulelist = zeros(size(r_pre,1)*size(r_pre,2),5);
k = 1;
for i = 1:size(r_pre,1)
    for j = 1:size(r_pre,2)
        rulelist(k,:) = [i,j,r_pre(i,j),1,1];
        k = k + 1;
    end
end
a = addrule(a, rulelist);

a1 = setfis(a,'DefuzzMethod','centroid');
writefis(a1,'simple');
a2 = readfis('simple');
 
figure;
plotfis(a2);
figure;
plotmf(a,'input',1);
figure;
plotmf(a,'input',2);
figure;
plotmf(a,'output',1);

showrule(a);
ruleview('simple');

%% 系统仿真
e = 0;
de = 0;
ke = 1;
kd = 1;
ku = 1;
for k = 1:N
    e1 = ke*e;
    de1 = kd*de;
    if e1>=2
        e1 = 2;
    elseif e1<=-2
        e1 = -2;
    end
    if de1>=2
        de1=2;
    elseif de1<=-2
        de1=-2;
    end
    in = [e1,de1];
    u = ku*evalfis([in], a2) + R(k);
    uu(1,k) = u;
    
    x = G*x + H*u;
    y = C*x;
    yy(1,k) = y;
    
    e_next = y - R(1,k);
    de = (e_next - e)/T;
    e = e_next;
end

kk = T:T:N*T;
yc = step(num,den,T:T:N*T);

%% 控制效果图
figure;
plot(kk, R, 'k',kk, uu, 'b-.', kk, yy, 'r', 'linewidth',1);
legend('系统期望输出量','模糊控制器输出量（二阶系统输入量）','系统输出量')
axis([0 20 0 3])
grid on 

%系统阶跃响应图
figure;
plot(kk,yc,'g--','linewidth',2);
xlabel('T')
axis([0 10 0 2])
grid on



        