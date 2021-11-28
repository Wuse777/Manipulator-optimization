# -*- coding: utf-8 -*- #
 
# ------------------------------------------------------------------
# File Name:        GA.py
# Author:           Yuanhai Huang
# Version:          1.0
# Created:          2021/11/28
# Description:      Main Function:    performing genetic algorithm 
#                   Thanks to scikit-opt with genetic algorithm. 
#                   However, as the low efficiency for the target function, it costs much more time to gain the result.
#                   Here we give an example on what we suppose to do.
#                   
# History:
#       <author>        <version>       <time>  
#
# ------------------------------------------------------------------
from sko.GA import GA
from sko.tools import set_run_mode
from optimization_model import fitness

import pandas as pd
import matplotlib.pyplot as plt

lb=[100,200,200,80,80,80]
ub=[140,600,600,120,120,120]

constraint_eq=[lambda x:x[0]+x[1]+x[2]+x[4]-1008]

set_run_mode(fitness,'parallel')

print('start running')
ga=GA(func=fitness,n_dim=6,size_pop=20,max_iter=10,prob_mut=0.001,lb=lb,ub=ub,constraint_eq=constraint_eq)
best_x, best_y = ga.run()
print('best_x:', best_x, '\n', 'best_y:', best_y)

Y_history = pd.DataFrame(ga.all_history_Y)
fig, ax = plt.subplots(2, 1)
ax[0].plot(Y_history.index, Y_history.values, '.', color='red')
Y_history.min(axis=1).cummin().plot(kind='line')
plt.show()