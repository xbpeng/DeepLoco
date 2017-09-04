import os
import subprocess

exe_path = 'TerrainRL_Optimizer.exe'
args = '-arg_file= args/opt_gen_perf_test.txt'

blend0 = 0
blend1 = 1
num_steps = 10

for i in range(0, num_steps):
    curr_blend = blend0 + (blend1 - blend0) * i / (num_steps - 1)
    
    command = exe_path + ' ' + args
    command += ' ' + '-terrain_blend=' + ' ' + str(curr_blend)
    
    print(command + '\n')
    subprocess.call(command)
