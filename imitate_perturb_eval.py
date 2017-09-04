import os
import subprocess

exe_path = 'TerrainRL_Optimizer_2d.exe'
args = '-arg_file= args/opt_imitate_perturb_eval.txt'

min_perturb = 0
max_perturb = 200 # biped
#max_perturb = 400 # dog raptor
num_perturb_steps = 11

for p in range(num_perturb_steps):
    lerp = p / (num_perturb_steps - 1.0)
    perturb = lerp * (max_perturb - min_perturb) + min_perturb
    
    command = exe_path + ' ' + args
    command += ' ' + '-min_perturb=' + ' ' + str(perturb)
    command += ' ' + '-max_perturb=' + ' ' + str(perturb)
    command += ' ' + '-terrain_blend=' + ' ' + str(lerp)
    
    print(command + '\n')
    subprocess.call(command)
