import os
import subprocess

hlc_files_dir = 'output/intermediate/hlc_policy/'
llc_files_dir = 'output/intermediate/llc_policy/'
exe_path = 'TerrainRL_Optimizer.exe'

args = '-arg_file= args/opt_int_hike_eval.txt'

root_dir = curr_dir =os.path.dirname(os.path.abspath(__file__))
print ("Root Directory" + str(root_dir))

os.chdir(root_dir)
hlc_files = os.listdir(hlc_files_dir)
llc_files = os.listdir(llc_files_dir)

hlc_actor_files = []
hlc_critic_files = []
for f in hlc_files:
    filename, ext = os.path.splitext(f)
    if (ext == '.h5'):
        if '_critic' in filename:
            hlc_critic_files.append(f)
        else:
            hlc_actor_files.append(f)

llc_actor_files = []
llc_critic_files = []
for f in llc_files:
    filename, ext = os.path.splitext(f)
    if (ext == '.h5'):
        if '_critic' in filename:
            llc_critic_files.append(f)
        else:
            llc_actor_files.append(f)
            

num_hlc_actor_files = len(hlc_actor_files)
num_hlc_critic_files = len(hlc_critic_files);
num_llc_actor_files = len(llc_actor_files)
num_llc_critic_files = len(llc_critic_files);

print('hlc_model files:')
for f in range(0, num_hlc_actor_files):
    curr_str = hlc_actor_files[f];
    if (num_hlc_critic_files > f):
        curr_str += ' ' + hlc_critic_files[f]
    print(curr_str)
print('\n\n')

print('llc_model files:')
for f in range(0, num_llc_actor_files):
    curr_str = llc_actor_files[f];
    if (num_llc_critic_files > f):
        curr_str += ' ' + llc_critic_files[f]
    print(curr_str)
print('\n')


#num_files = max(num_hlc_actor_files, num_llc_actor_files)
num_files = num_hlc_actor_files
f_steps = 1
hlc_actor_files.sort()
hlc_critic_files.sort()
llc_actor_files.sort()
llc_critic_files.sort()

for f in range(0, num_files, f_steps):
    command = exe_path + ' ' + args

    #if (num_llc_actor_files > f):
        #command += ' ' + '-policy_model=' + ' ' + llc_files_dir + llc_actor_files[f]
    #if (num_llc_critic_files > f):
        #command += ' ' + '-critic_model=' + ' ' + llc_files_dir + llc_critic_files[f]
    if (num_hlc_actor_files > f):
        command += ' ' + '-policy_model1=' + ' ' + hlc_files_dir + hlc_actor_files[f]
    if (num_hlc_critic_files > f):
         command += ' ' + '-critic_model1=' + ' ' + hlc_files_dir + hlc_critic_files[f] 
    print(command + '\n')
    subprocess.call(command)
