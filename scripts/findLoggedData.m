function log_data=findLoggedData(from_day,binpath)
if nargin<2
  binpath='~/.ros/';
end
if nargin<1
  from_day=30;
end

allfiles=dir(binpath);
log_data=[];
for iF=1:length(allfiles)
  idx=strfind(allfiles(iF).name,'_JointState_');
  if not(isempty(idx))
    if (now-datenum(allfiles(iF).datenum))<from_day
      test_data=table;
      test_data.test_name={allfiles(iF).name(1:idx-1)};
      test_data.complete_name={allfiles(iF).name};
      test_data.type={'JointState'};
      test_data.topic={allfiles(iF).name(idx+length('_JointState_'):end)};
      log_data=[log_data;test_data];
    end
  end
  
end