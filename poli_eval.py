import os
import subprocess
import sys

poli_files_dir = sys.argv[2] 
exe_path = 'x64/Release/TerrainRL_Optimizer'

args = '-arg_file= ' + sys.argv[1] 
root_dir = curr_dir =os.path.dirname(os.path.abspath(__file__))
print ("Root Directory" + str(root_dir))

os.chdir(root_dir)
files = os.listdir(poli_files_dir)

actor_files = []
critic_files = []
fd_files = []
for f in files:
    filename, ext = os.path.splitext(f)
    if (ext == '.h5'):
        if '_critic' in filename:
            critic_files.append(f)
        elif '_forward_dynamics' in filename:
            fd_files.append(f)
        else:
            actor_files.append(f)

num_files = len(actor_files)
num_critic_files = len(critic_files);
num_fd_files = len(fd_files);

print('model files:')
for f in range(0, num_files):
    curr_str = actor_files[f];
    if (num_critic_files > f):
        curr_str += ' ' + critic_files[f]
    print(curr_str)
print('\n')


num_files = len(actor_files)
f_steps = 1
actor_files.sort()
critic_files.sort()
fd_files.sort()

for f in range(0, num_files, f_steps):
    command = exe_path + ' ' + args
    command += ' ' + '-policy_model=' + ' ' + poli_files_dir + actor_files[f]

    if (num_critic_files > f):
         command += ' ' + '-critic_model=' + ' ' + poli_files_dir + critic_files[f]
         
    if (num_fd_files > f):
         command += ' ' + '-forward_dynamics_model=' + ' ' + poli_files_dir + fd_files[f]
        
    print(command + '\n')
    subprocess.call(command, shell=True)
