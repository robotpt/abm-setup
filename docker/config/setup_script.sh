#!/bin/bash

echo 'Hello, master'

source /opt/ros/kinetic/setup.bash 
source /root/ws/catkin_ws/devel/setup.bash

alias gs="git status"
alias gl="git log --pretty=oneline --graph"
alias gb="git branch -a"
alias gg="git gui"
alias rh="roscd; cd .."
alias rs="roscd; cd ../src"
alias cm="roscd; cd ..; catkin_make"
alias code="code --user-data-dir /root/.visual_code/"
