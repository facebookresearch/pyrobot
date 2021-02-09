---
id: contributing
title: Contributing to PyRobot
sidebar_label: Contributing to PyRobot
---




## Pull Request Process
1.) Ensure any install or build dependencies are removed before the end of the layer when doing a build.
2.) Update the README.md with details of changes to the interface, this includes new environment variables, exposed ports, useful file locations and container parameters.
3.) Increase the version numbers in any examples files and the README.md to the new version that this Pull Request would represent. The versioning scheme we use is SemVer.
4.) You may merge the Pull Request in once you have the sign-off of two other developers, or if you do not have permission to do that, you may request the second reviewer to           merge it for you.

 ## Our Pledge
 
 In the interest of fostering an open and welcoming environment, we as contributors and maintainers pledge to making participation in our project and our community a harassment-free experience for everyone, regardless of age, body size, disability, ethnicity, gender identity and expression, level of experience, nationality, personal appearance, race, religion, or sexual identity and orientation.
 
 ## Our Standards
 
 Examples of behavior that contributes to creating a positive environment include:

**Using welcoming and inclusive language

1.) Being respectful of differing viewpoints and experiences
2.) Gracefully accepting constructive criticism
3.) Focusing on what is best for the community
4.) Showing empathy towards other community members
5.)  Examples of unacceptable behavior by participants include:

6.) The use of sexualized language or imagery and unwelcome sexual attention or advances
7.) Trolling, insulting/derogatory comments, and personal or political attacks
8.) Public or private harassment
9.) Publishing others' private information, such as a physical or electronic address, without explicit permission
10.) Other conduct which could reasonably be considered inappropriate in a professional setting

## Scope
This Code of Conduct applies both within project spaces and in public spaces when an individual is representing the project or its community. Examples of representing a project or community include using an official project e-mail address, posting via an official social media account, or acting as an appointed representative at an online or offline event. Representation of a project may be further defined and clarified by project maintainers.

## Enforcement
Instances of abusive, harassing, or otherwise unacceptable behavior may be reported by contacting the project team at [INSERT EMAIL ADDRESS]. All complaints will be reviewed and investigated and will result in a response that is deemed necessary and appropriate to the circumstances. The project team is obligated to maintain confidentiality with regard to the reporter of an incident. Further details of specific enforcement policies may be posted separately.

Project maintainers who do not follow or enforce the Code of Conduct in good faith may face temporary or permanent repercussions as determined by other members of the project's leadership.

## Attribution

This Code of Conduct is adapted from the Contributor Covenant, version 1.4, available at http://contributor-covenant.org/version/1/4


Thank you for your interest in contributing to PyRobot! Your contributions will probably fall into two categories:

1. You want to implement a feature or bug-fix for an outstanding issue.
    - You can search for existing issues here: [https://github.com/facebookresearch/pyrobot/issues](https://github.com/facebookresearch/pyrobot/issues)
    - Pick an issue and comment on the task that you want to work on this feature.
    - If you need more context on a particular issue, please ask and we shall provide.
2. You would like to propose a new feature and implement it.
    - Post about your intended feature as a Github issue, and we shall discuss the design and
    implementation. Once we agree that the plan looks good, go ahead and implement it. Note that any core changes such as to the core PyRobot code or LoCoBot driver code will take a longer review process.

Once you finish implementing a feature or bug-fix, please send a Pull Request to
[https://github.com/facebookresearch/pyrobot](https://github.com/facebookresearch/pyrobot)

For more best practices on how to contribute, please see [Contributing Guide](https://github.com/pytorch/pytorch/blob/master/docs/source/community/contribution_guide.rst) from the [PyTorch](https://pytorch.org) repo. The following document covers some of the more technical aspects of contributing to PyRobot.

## Issues
We use GitHub issues to track public bugs. Please ensure your description is
clear and has sufficient instructions to be able to reproduce the issue.

Facebook has a [bounty program](https://www.facebook.com/whitehat/) for the safe
disclosure of security bugs. In those cases, please go through the process
outlined on that page and do not file a public issue.

## Codebase Structure

* [docs](docs) This folder is for tutorial documentation
	* [website/docs](docs/website/docs) Contains the tutorial markdown files for each tutorial in [pyrobot.org](https://www.pyrobot.org/docs/overview).
* [examples](examples)
	* [locobot](examples/locobot) Examples for commanding low level primitive actions for LoCobot
	* [sawyer](examples/sawyer) Commanding low level primitive actions for the sawyer arm
	* Contains other robot learning examples like [grasping](examples/grasping) and [sim2real](examples/sim2real)
* [robots](robots)
	* [LoCoBot](robots/LoCoBot) Drivers and ROS packages for the real LoCoBot and Gazebo simulator
		* [install](robots/LoCoBot/install) bash scripts for easy installation
		* [locobot_calibration](robots/LoCoBot/locobot_calibration) ROS package for hand-eye camera calibration for LoCoBot
		* [locobot_control](robots/LoCoBot/locobot_control) ROS package for LoCoBot drivers, teleoperation, inverse kinematics, etc.
		* [locobot_description](robots/LoCoBot/locobot_description) ROS package with LoCoBot urdf, mesh files, etc.
		* [locobot_gazebo](robots/LoCoBot/locobot_gazebo) ROS package for LoCoBot gazebo interface
		* [locobot_lite_moveit_config](robots/LoCoBot/locobot_lite_moveit_config) MoveIt package specific to full LoCoBot hardware setup (arm+base+camera mount)
		* [locobot_moveit_config](robots/LoCoBot/locobot_moveit_config) MoveIt package specific to full LoCoBot-lite hardware setup
		* [locobot_navigation](robots/LoCoBot/locobot_navigation)
			* [orb_slam2_ros](robots/LoCoBot/locobot_navigation/orb_slam2_ros) ROS package for ORB-SLAM2
			* [base_navigation](robots/LoCoBot/locobot_navigation/base_navigation) MoveBase package specific to LoCoBot
		* [thirdparty](robots/LoCoBot/thirdpart) Folder for LoCoBot specific external submodules
* [tests](tests) Tests for the core PyRobot API



## Testing
If you've added code that should be tested, please add tests. You can either add to existing test files or create new ones in the [tests](tests/) folder. The [tests/run_tests.py](tests/run_tests.py) script will launch and run the tests on a gazebo instance of the LoCoBot robot. Please also run existing tests and add new ones for the real robot (you can pass in *test_real* flag to the script). If you do not have any tests, please explain the reason.

## Tutorials
Please also consider adding tutorials if you are planning to submit a major feature, application or algorithm. Almost all the scripts in the [examples](examples) folder have their associated [tutorials](https://www.pyrobot.org/docs/overview) synced with [pyrobot.org](https://www.pyrobot.org/docs/overview). To add a new tutorial, first create a markdown file in [docs/website/docs](docs/website/docs) or add a symlink to wherever you have added it in the repo (we suggest having a single location for this markdown file for better management). If you want to add a tutorial, you also need to declare the markdown file *id* in [docs/website/website/sidebars.json](docs/website/website/sidebars.json) so that it shows up on the website.

## Contributor License Agreement ("CLA")
In order to accept your pull request, we need you to submit a CLA. You only need
to do this once to work on any of Facebook's open source projects.

Complete your CLA here: <https://code.facebook.com/cla>

## Community
Please consider joining our forum (start a google group?)

## Coding Style

The python code uses [Flake8](https://pypi.org/project/flake8/) style. [autopep8](https://github.com/hhatto/autopep8) will come in handy.

## License
By contributing to **PyRobot**, you agree that your contributions will be licensed
under the LICENSE file in the root directory of this source tree.
