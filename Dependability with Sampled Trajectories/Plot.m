clc;
close all;
clear all;

%  Author:   Yi Dong
%  University of Liverpool
%  Date: 12/SEP/2021


Safety_property = [1 1 1 1 1 1 1 1 0.9722 0.9751 0.9770 0.9790 0.9807 0.9821 0.9832 0.9841 0.9850 0.9856 0.9863 0.9869 0.9935 0.9914 0.9935 0.9948];
R1 = [7492 7342 7257 7192 7406 7412 7363 7337 7032 7065 7071 7109 7159 7183 7216 7213 7208 7198 7173 0.7166 0.7246 0.7185 0.7213 0.7229];
R2 = [7492 7342 7257 7192 7406 7412 7363 7337 7310 7314 7301 7319 7353 7362 7384 7372 7359 7342 7309 0.7297 0.7311 0.7271 0.7278 0.7281];
Robustness = R1./R2;
Successrate = [5/5 10/10 15/15 20/20 25/25 30/30 35/35 40/40 44/45 49/50 54/55 59/60 64/65 69/70 74/75 79/80 84/85 89/90 94/95 99/100 199/200 298/300 398/400 498/500];
% R2 = [0.8522 0.8920 0.8960 0.9244 0.9019 0.8397];
% R3 = [0.6567 0.7645 0.7970 0.8544 0.8467 0.8124];
Detection = 1- [147 156 162 166 153 153 156 157 159 159 159 158 156 156 154 155 156 157 159 159.42 158.56 161.12 160.62 160.48]/560;
Resilience = 1- [929 939 922 909 938 926 927 918 889 888 891 898 903 903 908 912 913 916 911 914.25 916.42 910.55 913.39 914.52]/(560*3);

figure('Units','centimeter','Position',[5 5 40 22.5]);
plot(Safety_property,'b-^', 'linewidth',2,'MarkerSize',10); hold on;
plot(Robustness,'r--o', 'linewidth',2,'MarkerSize',10);hold on;
% plot(R2,'k-*', 'linewidth',2,'MarkerSize',10);hold on;
plot(Detection,'g-+', 'linewidth',2,'MarkerSize',10); hold on;
plot(Resilience,'m--s', 'linewidth',2,'MarkerSize',10);hold on;
plot(Successrate,'k--s', 'linewidth',2,'MarkerSize',10);hold on;
set(gca,'fontsize',30,'position',[0.1,0.13,0.85,0.8])
axis([0 25 0 1])
% legend('Safety','Robustness','Detection','Resilience','Mission Success Rate','fontsize',18,'interpreter','latex','Location','SouthEast');
% title('Properties with Different Testing Episodes','fontsize',36,'interpreter','latex');
xlabel('Number of Testing Trajectories','fontsize',36,'interpreter','latex'),ylabel('Dependability Properties','fontsize',36,'interpreter','latex');
xticks([0,1,5,10,15,20,21,22,23,24,25])
xticklabels({'','5','25','50','75','100','200','300','400','500'})

saveas(gcf,'MCsafety.eps','epsc')
