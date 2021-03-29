
%/*********************************************************************
% *
% * Authors: Manuel Beschi (manuel.beschi@itia.cnr.it)
% *          Enrico Villagrossi (enrico.villagrossi@itia.cnr.it)
% *          
% *
% * Software License Agreement (BSD License)
% *
% *  Copyright (c) 2016, National Research Council of Italy, Institute of Industrial Technologies and Automation
% *  All rights reserved.
% *
% *  Redistribution and use in source and binary forms, with or without
% *  modification, are permitted provided that the following conditions
% *  are met:
% *
% *   * Redistributions of source code must retain the above copyright
% *     notice, this list of conditions and the following disclaimer.
% *   * Redistributions in binary form must reproduce the above
% *     copyright notice, this list of conditions and the following
% *     disclaimer in the documentation and/or other materials provided
% *     with the distribution.
% *   * Neither the name of the National Research Council of Italy nor the 
% *     names of its contributors may be used to endorse or promote products 
% *     derived from this software without specific prior written permission.
% *
% *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
% *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
% *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
% *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
% *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
% *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
% *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% *  POSSIBILITY OF SUCH DAMAGE.
% *********************************************************************/

function data=bin_convert(filename,n_cols,burn_after_reading)
% data=bin_convert(filename,n_cols,burn_after_reading)
%
% filename: file name (with extension)
% n_cols: number of columns
% burn_after_reading: delete file after reading (default: 0)

if (nargin<3)
  burn_after_reading=0;
end

f=dir(filename);
if isempty(f)
  error(sprintf('no file %s exist',filename))
end
fid=fopen(filename);
if (f.bytes==0)
    data=[];
    return;
end
n_rows=f.bytes/8/n_cols;
data=fread(fid,[n_cols,n_rows],'double')';
fclose(fid);

if sum(diff(data(:,1))<0)>0
  warning('Time vector is not monotinic');
  warning('is the subscriber topic a variable-size signal? (NOT SUPPORTED YET)')
  error('Time vector errors')
end

% exclude duplicates
data=data(diff(data(:,1))>0,:);

if burn_after_reading
  if isunix
    system(['rm ',filename]);
  else
    fprintf('not supported yet\nCheck system command to delete files on Windows\n');
  end
end
