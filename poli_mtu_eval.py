import os
import subprocess

poli_files_dir = 'output/intermediate/'
exe_path = 'TerrainRL_Optimizer.exe'

args = '-arg_file= args/opt_int_imitate_mtu_eval.txt'

root_dir = curr_dir =os.path.dirname(os.path.abspath(__file__))
print('Root Directory' + str(root_dir))

os.chdir(root_dir)
files = os.listdir(poli_files_dir)

actor_files = []
critic_files = []
ctrl_files = []

for f in files:
    filename, ext = os.path.splitext(f)
    if (ext == '.h5'):
        if '_critic' in filename:
            critic_files.append(f)
        else:
            actor_files.append(f)
    elif (ext == '.txt'):
        if 'int_mtus' in filename:
            ctrl_files.append(f)

num_actor_files = len(actor_files)
num_critic_files = len(critic_files)
num_ctrl_files = len(ctrl_files)
min_num_files = min(num_ctrl_files, min(num_actor_files, num_critic_files))

print('model files:')
for f in range(0, min_num_files):
    curr_str = actor_files[f];
    curr_str += ' ' + critic_files[f]
    curr_str += ' ' + ctrl_files[f]
    print(curr_str)
print('\n')


num_files = len(actor_files)
f_steps = 1

for f in range(0, min_num_files, f_steps):
    command = exe_path + ' ' + args
    command += ' ' + '-policy_model=' + ' ' + poli_files_dir + actor_files[f]
    command += ' ' + '-critic_model=' + ' ' + poli_files_dir + critic_files[f]
    command += ' ' + '-char_ctrl_param_file=' + ' ' + poli_files_dir + ctrl_files[f]
         
    print(command + '\n')
    subprocess.call(command)
