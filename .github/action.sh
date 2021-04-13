#!/usr/bin/env bash

echo -c 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
export WORKSPACE=${HOME}/ros2/${ROS_DISTRO}/src

echo "Workspace: ${WORKSPACE}"
echo "Repo name: ${REPO_NAME}"
echo "Additonal repo name: ${ADD_REPO_NAME}"

if [ ! -d $WORKSPACE ];
then
    mkdir -p $WORKSPACE
fi

# setup the workspace for building
echo "Symlink of workspace ${WORKSPACE}"
cp -r ${HOME}/$REPO_NAME $WORKSPACE/
cp -r ${HOME}/$ADD_REPO_NAME $WORKSPACE/

echo -c "Building workspace"
cd $WORKSPACE/..
colcon build --symlink-install

source ./install/setup.bash
colcon test --event-handlers console_direct+ --packages-select node_registry

colcon test-result --all
