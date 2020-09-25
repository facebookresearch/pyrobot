echo 'Removing *pyro* from ~/'
sudo rm -rf ~/*pyro*
echo 'Removing *pyro* from ~/workspaces'
sudo rm -rf ~/workspaces/*pyro*
echo 'Removing *pyro* from ~/pyenvs'
sudo rm -rf ~/pyenvs/*pyro*
echo 'Cleaning ~/.bashrc'
sed -i '/pyro/d' ~/.bashrc
echo 'Pyrobot Install Purged!'
