arg_list = argv();
# octave -qf comparing_multipol_continuos.m /home/renatargo/Dropbox/phd_grl_results/phd_grl_mpol_results/mpol_dpg_20_density_data 10
#str = "/home/renatargo/Dropbox/phd_grl_results/phd_grl_mpol_results/mpol_dpg_20_density_data";
#n = 10;
for i=0:(length(arg_list)/2 -1)
  str = arg_list{2*i + 1}
  n = str2num(arg_list{2*i + 2})
  maximum = -inf
  sum = load([str "-0.txt"])(:,3);
  for i = 1:(n-1)
    sum = sum + load([str "-" num2str(i) ".txt"])(:,3);
  endfor
  mean = sum/n;
  maximum = max(max(mean), maximum);
    
  mean_dpg_20_density = mean; #print_mean(str, 10)
  max_dpg_20_density = maximum; #print_mean(str, 10)

  x = 1:length(mean);
  plot(x,mean_dpg_20_density, [";mean.dpg.20.density(" num2str(max_dpg_20_density) ");"], "markersize", 10);
endfor

pause()

#plot(x,mean_density, ';mean.density;', "markersize", 10,
#     x,mean_density2, ';mean.density2;', "markersize", 5,
#     x,mean_dpg, ';mean.dpg;', "markersize", 2,
#     x, mean_ac_tc, ';mean.ac.tc;', "markersize", 6,
#     x, mean_dpg4, ';mean.dpg4;', "markersize", 7);
     