# fallback_decision

$mkdir ~/{your workspace}

$cd ~/{your workspace}

$mkdir src/

$catkin_make

$cd src/

$git clone https://github.com/Hansaem-Jeong/fallback_decision.git

$mkdir fallback_decision/include

$cd ../

$catkin_make

$source devel/setup.bash

$rosrun fallback_decision fallback_decision

--- in other terminal

$rosbag play ~/{your workspace}/src/fallback_decision/bagfile/aes_data_bagfile.bag

======================================================

sudo apt-get install libmagick++-dev
sudo apt-get install graphicsmagick-libmagick-dev-compat
sudo apt-get install libgraphicsmagick1-dev
