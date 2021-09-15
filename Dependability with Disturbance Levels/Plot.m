clc;
close all;
clear all;

%  Author:   Yi Dong
%  University of Liverpool
%  Date: 12/SEP/2021


Safety_property = [0.9973 0.9942 0.9819 0.9530 0.8345 0.8106 0.6941 0.6846 0.6086 0.5561 0.6024 0.5293 0.5354 0.4783 0.4643 0.4433 0.4292 0.4111 0.3996 0.4626 0.3923];
R1 = [7256 7913 8419 8285 7395 7213 6107 6093 5303 4848 5195 4582 4689.5 4107 3951 3827 3652 3495 3395 3906 3216];
R2 = [7282 7972 8600 8753 9048 9100 9139 9223 9171 9243 9124 9223 9278 9231 9224 9285 9238 9232 9296 9189.7 9092];% R2 = [0.8522 0.8920 0.8960 0.9244 0.9019 0.8397];
Robust = R1./R2;
Detection = 1- [160.5 118 80 71 54 51 47.5 42.82 44.586 40.523 47.28 40.43 37.7843 38.457375 39.33 34.429558 36.3684 35 34 40.9397527 40.2]/560;
Resilience = 1-[915.76 698.53 731.76 726.99 667.04 661.43 566.49 578.187 497.12 468.75 486.66 443.85 456.84 405.75 394.17 388.09 371.81 353.89 347.18 385.16 327.10]/(560*3);
Successrate = [300*1000/1002 295 289 281 235 230 184 186 151 131 148 121 127 107 104 91 85 83 74 96 73]/300;
figure('Units','centimeter','Position',[5 5 40 22.5]);
% plot(Safety_property,'b-^', 'linewidth',2); hold on;
% plot(Robust,'r--o', 'linewidth',2);hold on;
% plot(Detection,'k-*', 'linewidth',2);hold on;
% plot(Resilience,'m--s', 'linewidth',2);hold on;
plot(Safety_property,'b-^', 'linewidth',2,'MarkerSize',10); hold on;
plot(Robust,'r--o', 'linewidth',2,'MarkerSize',10);hold on;

% % plot(R2,'k-*', 'linewidth',2,'MarkerSize',10);hold on;
plot(Detection,'g-+', 'linewidth',2,'MarkerSize',10); hold on;
plot(Resilience,'m--s', 'linewidth',2,'MarkerSize',10);hold on;
plot(Successrate,'k--s', 'linewidth',2,'MarkerSize',10);hold on;
% plot(R1/10000,'c--o', 'linewidth',2,'MarkerSize',10);hold on;
set(gca,'fontsize',30,'position',[0.1,0.13,0.85,0.8])
axis([0 22 0.2 1])
% legend('Safety','Robustness','Detection','Resilience','Mission Success Rate','fontsize',18,'interpreter','latex','Location','SouthWest');
% title('Properties with Different Training Episodes','fontsize',36,'interpreter','latex');
% title('Properties with Different Environments','fontsize',36,'interpreter','latex');
xlabel('Variance of Disturbance','fontsize',36,'interpreter','latex'),ylabel('Dependability Properties','fontsize',36,'interpreter','latex');
xticks([0,1,3,5,7,9,11,13,15,17,19,21])
xticklabels({'','0','0.2','0.4','0.6','0.8','1.0','1.2','1.4','1.6','1.8','2.0'})

saveas(gcf,'SRsafety.eps','epsc')
