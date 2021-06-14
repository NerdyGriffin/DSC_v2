function [data_array] = readSerialData(serial_line, n_vars = 1)
%READSERIALDATA Summary of this function goes here
%   Detailed explanation goes here
data_str = split(serial_line,',');
% n_vars = 1; % number of variables printed
data_array = zeros(n_vars,1);
for ii=1:n_vars
    data_array(ii) = str2double(data_str(ii));
end
end

