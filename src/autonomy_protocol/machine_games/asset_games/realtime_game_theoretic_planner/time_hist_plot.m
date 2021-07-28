
figure(1)
clf;
hold on;
histogram(nash_times_k1_n4(nash_times_k1_n4>0)...
        ,'BinWidth',3 ...
        ,'DisplayName','K=1' ...
        );
histogram(nash_times_k2_n4(nash_times_k2_n4>0)...
        ,'BinWidth',3 ...
        ,'DisplayName','K=2' ...
        );
h=histogram(nash_times_k3_n4(nash_times_k3_n4>0)...
        ,'BinWidth',3 ...
        ,'DisplayName','K=3' ...
        );

ax = h.Parent;
chi = ax.Children;
%set(gca, 'Children',flipud(chi));

title('Time to Capture for N=4')
xlabel('Time (sec)');
ax = gca;
chi = ax.Children;
set(gca, 'Children',flipud(chi));
legend show;

disp([' mean capture time K=1, N=4 ' num2str(...
mean(nash_times_k1_n4(nash_times_k1_n4>0)) ) ]);
idx = velmatch_times>0 & nash_times_k1_n4>0 ;
disp([' time delta' num2str(...
mean(nash_times_k1_n4(idx)-velmatch_times(idx))   )]);

disp([' mean capture time K=2, N=4 ' num2str(...
mean(nash_times_k2_n4(nash_times_k2_n4>0)) ) ]);
idx = velmatch_times>0 & nash_times_k2_n4>0 ;
disp([' time delta' num2str(...
mean(nash_times_k2_n4(idx)-velmatch_times(idx))   )]);

disp([' mean capture time K=3, N=4 ' num2str(...
mean(nash_times_k3_n4(nash_times_k3_n4>0)) ) ]);
idx = velmatch_times>0 & nash_times_k3_n4>0 ;
disp([' time delta' num2str(...
mean(nash_times_k3_n4(idx)-velmatch_times(idx))   )]);


figure(2)
clf;
hold on;

histogram(nash_times_k2_n4(nash_times_k2_n4>0)...
        ,'BinWidth',3 ...
        ,'DisplayName','N=4' ...
        );
histogram(nash_times_k2_n6(nash_times_k2_n6>0)...
        ,'BinWidth',3 ...
        ,'DisplayName','N=6' ...
        );
histogram(nash_times_k2_n8(nash_times_k2_n8>0)...
        ,'BinWidth',3 ...
        ,'DisplayName','N=8' ...
        );
h=histogram(nash_times_k2_n10(nash_times_k2_n10>0)...
        ,'BinWidth',3 ...
        ,'DisplayName','N=10' ...
        );

bounds = xlim;
ax = h.Parent;
chi = ax.Children;
set(gca, 'Children',flipud(chi));
legend show;
title('Time to Capture for K=2')
xlabel('Time (sec)');

disp(' ');
disp([' mean capture time K=2, N=4 ' num2str(...
mean(nash_times_k2_n4(nash_times_k2_n4>0)) ) ]);
idx = velmatch_times>0 & nash_times_k2_n4>0 ;
disp([' time delta' num2str(...
mean(nash_times_k2_n4(idx)-velmatch_times(idx))   )]);

disp([' mean capture time K=2, N=6 ' num2str(...
mean(nash_times_k2_n6(nash_times_k2_n6>0)) ) ]);
idx = velmatch_times>0 & nash_times_k2_n6>0 ;
disp([' time delta' num2str(...
mean(nash_times_k2_n6(idx)-velmatch_times(idx))   )]);

disp([' mean capture time K=2, N=8 ' num2str(...
mean(nash_times_k2_n8(nash_times_k2_n8>0)) ) ]);
idx = velmatch_times>0 & nash_times_k2_n8>0 ;
disp([' time delta' num2str(...
mean(nash_times_k2_n8(idx)-velmatch_times(idx))   )]);

disp([' mean capture time K=2, N=10 ' num2str(...
mean(nash_times_k2_n10(nash_times_k2_n10>0)) ) ]);
idx = velmatch_times>0 & nash_times_k2_n10>0 ;
disp([' time delta' num2str(...
mean(nash_times_k2_n10(idx)-velmatch_times(idx))   )]);


figure(3)
clf;
hold on;
histogram(velmatch_times(velmatch_times>0)...
        ,'BinWidth',3 ...
        ,'DisplayName','N=4' ...
        );
xlim(bounds)
title('Velocity Matching Time to Capture');
xlabel('Time (sec)');

disp(' ');
disp([' mean capture time VM ' num2str(...
mean(velmatch_times(velmatch_times>0)) ) ]);