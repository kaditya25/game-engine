% convention: positive time represents capture at that time, 
%             negative time represents evader reaching target
% time greater than 20 means that the evader was 'chased off'  
%
% x = -10...0
% y = 2...5
%
% plots the comparative outcome:
%   +4 = NG catches and VM fails
%   +3 = NG catches faster than VM
%   +2 = NG end in stalemate, VM loses 
%   +1 = NG takes longer than VM for evader to reach target
%   0  = equal outcome
%   ... and negative values for the opposites
function [img] = capture_zone_plot(nash_times,velmatch_times,x,y)
  % fix for older data sets
  %nash_times(nash_times<-20)=0;
  %velmatch_times(velmatch_times<-20)=0;

  dims = size(nash_times);
  img = ones(dims)*NaN;
  for i=1:dims(1)
    for j=1:dims(2)
      n  = nash_times(i,j);
      vm = velmatch_times(i,j);
      if (x(i)==-8 && y(j)==3.5)
        %keyboard
      end
      if      (n==vm) % equal outcome
        img(i,j)=0;
      elseif  (n>0 && vm<0) % nash wins, vm loses
        img(i,j)=4;
      elseif  (n<0 && vm>0) 
        img(i,j)=-4;
      elseif  (n>0 && vm>0)
        if (n<vm)           % nash captures faster
          img(i,j)=3;
        else
          img(i,j)=-3;
        end
      elseif  (n<0 && vm<0)
        if (n<vm)           % nash delays asset for longer
          img(i,j)=1;
        else
          img(i,j)=-1;
        end
      elseif  (n==0)
        if (vm<0)
          img(i,j)=2;
        else 
          img(i,j)=0;
        end
      elseif  (vm==0)
        if (n<0)
          img(i,j)=-2;
        else
          img(i,j)=0;
        end
      else
        img(i,j)=NaN;
      end

    end
  end

  if (any(isnan(img),'all'))
    disp('error: case not accounted for!');
  end

  mymap = [ 1   0   0; %-4
            .75 0   0; %-3
            .50 0   0; %-2
            .25 0   0; %-1
            .2  .2  .2; %0
            0   .25 0; %1
            0   .50 0; %2
            0   .75 0; %3
            0   1.0 0;]; %4
            
  %figure(1);
  %imagesc([x(1),x(end)],[y(1),y(end)],rot90(img),[-3,3]);
  h=imagesc([x(1),x(end)],[y(1),y(end)],(img)',[-4,4]);
  %h=imagesc([x(1),x(end)],[y(1),y(end)],(nash_times)',[-3,3]);
  set(gca,'Ydir', 'normal')
  colormap(mymap)
  colorbar
  axis equal
  ylim([1.75+1/8,5.25-1/8])


end
