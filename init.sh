expected="src"
current="${PWD##*/}"

if [[ $current == $expected ]]; then
	echo "Initializing workspace ...\n"
	catkin_init_workspace
else
	echo "ERROR INITIALIZING WORKSPACE: this folder must be cloned as the src folder in a catkin workspace.\n"
fi
