% A = [];
% TT = zeros(5,5,1000);
% 
% for i = 0:999
%     name = "failure_level-"+string(i)+".csv";
%     A = csvread(name);
%     
%     for k = 1:size(A,1)-1
%         TT(A(k)+1,A(k+1)+1,i+1) = TT(A(k)+1,A(k+1)+1,i+1)+1;
%     end
% end
% 
% N = sum(TT,3);
% P = N./sum(N,2);
clear all
close all
clc

A = [];
TT = zeros(5,5,1000);

for i = 0:299
    name = "failure_level-"+string(i)+".csv";
    A = csvread(name);
    
    for k = 1:size(A,1)-1
        TT(A(k)+1,A(k+1)+1,i+1) = TT(A(k)+1,A(k+1)+1,i+1)+1;
    end
end

N = sum(TT,3);
P = N./sum(N,2);


writematrix(P,'MP0.csv') 
