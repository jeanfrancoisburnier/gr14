clear all 
close all
run('plot_map')
[x y]= extract_data('example.txt');
plot(x,y,'r','LineWidth',2)
