echo 'Removing *pyro* from ~/'
sudo rm -rf ~/*pyro*
echo 'Removing *pyro* from ~/workspaces'
sudo rm -rf ~/workspaces/*pyro*
echo 'Removing *pyro* from ~/py_envs'
sudo rm -rf ~/py_envs/*pyro*
echo 'Cleaning ~/.bashrc'
sed -i '/pyro/d' ~/.bashrc
echo 'Pyrobot Install Purged!'
