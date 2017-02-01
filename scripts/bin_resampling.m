
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

function data_out=bin_resampling(data_in,Ts)
% data_out=bin_resampling(data_in,Ts)
% 
% data_in: cell with bin matrices in format [time,data]
% Ts: desired sampling time
% data_out: cell with bin matrices in format [sampled_time,data]

tmin = zeros(length(data_in),1);
tmax = zeros(length(data_in),1);

for iC=1:length(data_in)
  tmin(iC) = data_in{iC}(1,   1);
  tmax(iC) = data_in{iC}(end, 1);
end

tmin = max(tmin);
tmax = min(tmax);

t = (tmin:Ts:tmax)';

for iC=1:length(data_in)
  data = interp1( data_in{iC}(:,1), data_in{iC}(:,2:end), t, 'linear', 'extrap' );
  data_out{iC} = [t data];
end
