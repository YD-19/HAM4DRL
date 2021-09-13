clc;
close all;
clear all;

%  Author:   Yi Dong
%  University of Liverpool
%  Date: 12/SEP/2021


Safety_property = [0.9315 0.7427 0.9771 1 0.9955 1 0.9946 0.9937 1 1 0.9799 1 1 0.9927 0.9859 0.9869 0.9862 0.9925 1 0.9922 0.9923 1 0.9849 0.9847 0.9953 0.9931 0.9710 0.9921 0.9925 0.9924];
R1 = [0.3846 0.4198 0.4726 0.5763 0.4210 0.5693 0.4634 0.5612 0.5433 0.5644 0.5700 0.5731 0.5808 0.5739 0.5098 0.5338 0.5079 0.5555 0.5589 0.5726 0.5642 0.5812 0.5465 0.5915 0.5803 0.5391 0.5477 0.5765 0.5698 0.5731];
R2 = [0.4530 0.6772 0.4954 0.5763 0.4255 0.5693 0.4687 0.5675 0.5433 0.5644 0.5901 0.5731 0.5808 0.5812 0.5239 0.5471 0.5217 0.5630 0.5589 0.5804 0.5719 0.5812 0.5616 0.6067 0.5949 0.5460 0.5767 0.5844 0.5774 0.5808];
Robustness = R1./R2;
Detection = 1- [321.31 194.87 312.02 260.15 353.79 261.32 333.53 265.63 282.80 265.59 249.29 261.35 254.09 253.72 290.25 278.74 292.49 266.78 270.65 253.73 259.74 253.49 266.09 237.22 245.60 277.18 254.90 249.04 254.75 250.76]/560;
Resilience = 1- [446 765 498 834 372 622 549 659 643 614 654 637 666 660 616 611 576 644 665 678 660 659 634 679 670 617 655 678 668 627]/(560*3);
Successrate = [0 0.3 0.94 0.77 0.92 0.99 0.99 0.99 1 1 0.96 1 1 0.99 0.98 0.98 0.98 0.99 1 0.99 0.98 0.99 0.98 0.98 0.97 0.99 0.96 0.99 0.99 0.99];
figure('Units','centimeter','Position',[5 5 40 22.5]);
% plot(Safety_property,'b-^', 'linewidth',2); hold on;
% plot(R1,'r--o', 'linewidth',2);hold on;
% plot(Detection,'k-*', 'linewidth',2);hold on;
% plot(Resilience,'m--s', 'linewidth',2);hold on;
plot(Safety_property,'b-^', 'linewidth',2,'MarkerSize',10); hold on;
plot(Robustness,'r--o', 'linewidth',2,'MarkerSize',10);hold on;
% plot(R2,'k-*', 'linewidth',2,'MarkerSize',10);hold on;
plot(Detection,'g-+', 'linewidth',2,'MarkerSize',10); hold on;
plot(Resilience,'m--s', 'linewidth',2,'MarkerSize',10);hold on;
plot(Successrate,'k--s', 'linewidth',2,'MarkerSize',10);hold on;
% plot(R1,'c--o', 'linewidth',2,'MarkerSize',10);hold on;

set(gca,'fontsize',30,'position',[0.1,0.13,0.85,0.8])
axis([0 31 0 1])
% legend('Safety','Robustness','Detection','Resilience','Mission Success Rate','fontsize',18,'interpreter','latex','Location','SouthEast');
% title('Properties with Different Training Episodes','fontsize',36,'interpreter','latex');
xlabel('Number of Training Episode','fontsize',36,'interpreter','latex'),ylabel('Dependability Properties','fontsize',36,'interpreter','latex');
xticks([0,1,5,9,13,17,21,25,29])
xticklabels({'','0','40','80','120','160','200','240','280'})

saveas(gcf,'Psafety.eps','epsc')
