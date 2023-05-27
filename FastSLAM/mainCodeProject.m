rosshutdown
clc
clear all
close all

setenv('ROS_MASTER_URI','http://robotvirtualbox:11311');
rosinit

FastSLAM([0 0 0]);