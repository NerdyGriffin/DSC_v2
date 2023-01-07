function [data_array] = parseSerialData(serial_str)
%PARSESERIALDATA Summary of this function goes here
%   Detailed explanation goes here
data_str = split(serial_str,',');
n_vars = length(data_str); % number of variables printed
data_array = zeros(n_vars,1);
parfor ii=1:n_vars
    data_array(ii) = str2double(data_str(ii));
end
end
