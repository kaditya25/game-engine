times_n4=[.000232,.00133,.00950,.168,6.41]
times_n8=[.000333 .00256 .0469 1.822]


f=figure(3);
clf; hold on;

scatter(1:5,times_n4,50,[0,.45,.75],'filled','DisplayName','N=4');
plot(1:5,times_n4,'--','LineWidth',2,'Color',[0,.45,.75],'DisplayName','N=4','HandleVisibility','off')

scatter(1:4,times_n8,50,[.85,.35,0],'filled','DisplayName','N=8');
plot(1:4,times_n8,'--','LineWidth',2,'Color',[.85,.35,0],'DisplayName','N=8','HandleVisibility','off')
ax = f.Children;
set(ax,'YScale','log');
legend('location','southeast') ;
title('Realtime Computation per Timestep');
xlabel('K');
ylabel('mean computation time (sec)');

f=figure(4);
clf; hold on;

times_k3=[ 0.00641 0.00950 0.0153 0.0216 0.0317 0.0469 0.0598 0.0798 0.102 0.132];
times_k3_N = 3:12;

times_k2=[ 0.00102 0.00161 0.00256 0.00360 0.00455 0.00585 0.00781 0.00950 0.0116];
times_k2_N = 4:2:20;

scatter(times_k2_N,times_k2,50,[0,.45,.75],'filled','DisplayName','K=2');
plot(times_k2_N,times_k2,'--','LineWidth',2,'Color',[0,.45,.75],'DisplayName','K=2','HandleVisibility','off')

scatter(times_k3_N,times_k3,50,[.85,.35,0],'filled','DisplayName','K=3');
plot(times_k3_N,times_k3,'--','LineWidth',2,'Color',[.85,.35,0],'DisplayName','K=3','HandleVisibility','off')

ax = f.Children;
set(ax,'YScale','log');
legend('location','southeast') ;
title('Realtime Computation per Timestep');
xlabel('N');
ylabel('mean computation time (sec)');

