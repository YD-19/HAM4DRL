clear all
clc
close all
A = [];
name = "crash-299.csv";
A = csvread(name);
    
size(find(A==2))

