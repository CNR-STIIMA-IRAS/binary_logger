
# MATLAB script 

** %%%%%%%%%%%%%%%%%% **

Example of usage (MATLAB pseudo code):

% binary_filename: path and name of the filename

% n_cols: number of the columns expected to unpack to binary file 

% burn_after_reading: true will destroy the original binary file

data = bin_convert( binary_filename, n_cols, burn_after_reading )


** %%%%%%%%%%%%%%%%%% **

% data: coming from the bin_convert function

% Ts: is the samplig time to be used for the resampling (NOTE: in case of multiple signals sampled to different sampling frequencies use the lower to avoid the signal oversampling)

data_out = bin_resampling( data, Ts )
 
