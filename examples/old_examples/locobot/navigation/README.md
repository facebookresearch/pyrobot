- Kobuki position control: `python examples/locobot/navigation/base_position_control.py --base_planner none --base_controller ilqr --smooth --close_loop --relative_position 1.,1.,1.57 --botname locobot`\
  <img src="runs/kobuki-position-control-smooth-close-1.png" height=250> <img src="runs/kobuki-position-control-smooth-close-1.gif" height=250>
- Kobuki position control: `python examples/locobot/navigation/base_position_control.py --base_planner none --base_controller ilqr --smooth --close_loop --relative_position 1.5,1.5,0 --botname locobot`\
  <img src="runs/kobuki-position-control-smooth-close-2.png" height=250> <img src="runs/kobuki-position-control-smooth-close-2.gif" height=250>
- Kobuki tracking twocircles in openloop: `python examples/locobot/navigation/base_trajectory_tracking.py --noclose_loop --type twocircles --botname locobot`
  <img src="runs/kobuki-twocircles-noclose.png" height=250> <img src="runs/kobuki-twocircles-noclose.gif" height=250>
- Kobuki tracking twocircle in closeloop: `python examples/locobot/navigation/base_trajectory_tracking.py --close_loop --type twocircles --botname locobot` \
  <img src="runs/kobuki-twocircles-close.png" height=250> <img src="runs/kobuki-twocircles-close.gif" height=250>
- Create tracking twocircles in openloop: `python examples/locobot/navigation/base_trajectory_tracking.py --noclose_loop --type twocircles --botname locobot`\
  <img src="runs/create-twocircles-close0.png" height=250>
- Create tracking twocircle in closeloop: `python examples/locobot/navigation/base_trajectory_tracking.py --close_loop --type twocircles --botname locobot`\
  <img src="runs/create-twocircles-close1.png" height=250>
