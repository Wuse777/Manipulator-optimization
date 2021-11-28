function state = savefun(options,state,flag)
%用于保存ga的中间数据
%   存入state structure 中的Population与score变量
% 
% writematrix(state.Population,"./data/population.xls","Sheet",num2str(state.Generation));
% 
% writematrix(state.Score,"./data/score.xls","sheet",num2str(state.Generation));


name=['./GA_data_' num2str(state.StartTime) '.xls'];


fprintf("第%s代计算完成\n",num2str(state.Generation));

writecell({'d1','a2','a3','d4','d5','d6','score','mean','best'},name,"Sheet",num2str(state.Generation),'Range','A1:I1');

writematrix(state.Population,name,"Sheet",num2str(state.Generation),'Range','A2');

writematrix(state.Score,name,"Sheet",num2str(state.Generation),'Range','G2');

writematrix(mean(state.Score),name,"Sheet",num2str(state.Generation),'Range','H2');

writematrix(min(state.Score),name,"Sheet",num2str(state.Generation),'Range','I2');


end