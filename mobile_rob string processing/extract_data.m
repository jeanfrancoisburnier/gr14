function [ time data ] = extract_data( file_name )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    file=fileread(file_name);
    file_data = str2num(file);
    time = file_data(:,1);
    data = file_data(:,2);
end

