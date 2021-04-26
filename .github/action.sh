#!/usr/bin/env bash

echo -c 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
export WORKSPACE=${HOME}/ros2/${ROS_DISTRO}/src

echo "Workspace: ${WORKSPACE}"
echo "Repo name: ${REPO_NAME}"

if [ ! -d $WORKSPACE ];
then
    mkdir -p $WORKSPACE
fi

# setup the workspace for building
echo "Symlink of workspace ${WORKSPACE}"
cp -r $ACTION_BUILD_DIR $WORKSPACE/

echo -c "Building workspace"
cd $WORKSPACE/..
colcon build --symlink-install --event-handlers console_direct+ | tee output.txt
cat output.txt | grep "failed:"
if [ $? -eq 0 ]
then
  echo "The build failed"
  exit 1
else
  echo "The build ran ok"
fi
source ./install/setup.bash

colcon test --event-handlers console_direct+ --packages-select node_registry

colcon test-result --all
