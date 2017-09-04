import os
import subprocess

poli_files_dir = 'data/intermediate/dog/ace_mixed_rand_init_noise_no_actor_bias/'
exe_path = 'TerrainRL_Optimizer.exe'

args = '-arg_file= args/opt_int_poli_info.txt'

root_dir = curr_dir = os.path.dirname(__file__)
print(root_dir)

os.chdir(root_dir)
files = os.listdir(poli_files_dir)

model_files = []
for f in files:
    filename, ext = os.path.splitext(f)
    if (ext == '.h5'):
        model_files.append(f)

print('model files:')
for f in model_files:
    print(f)
print('\n')

num_files = len(model_files)
f_steps = 5

for f in range(0, num_files, f_steps):
    file = model_files[f];
    action_file = 'output/int_poli_actions/actions_' + os.path.splitext(file)[0] + '.txt'
    state_file = 'output/int_poli_states/states_' + os.path.splitext(file)[0] + '.txt'
    
    command = exe_path + ' ' + args
    command += ' ' + '-policy_model=' + ' ' + poli_files_dir + file
    command += ' ' + '-action_output_file=' + ' ' + action_file
    command += ' ' + '-action_id_state_output_file=' + ' ' + state_file
    
    print(command + '\n')
    subprocess.call(command)
