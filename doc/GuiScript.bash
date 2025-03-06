source /opt/ros/humble/setup.bash
source $HOME/.local/etc/WhillNavi2Gui/whill_navi2_source/install/setup.bash
export PACKAGE_NAME=whill_navi2
export EXEC_PATH=$(ros2 pkg prefix $PACKAGE_NAME)/lib/python3.10/site-packages/$PACKAGE_NAME/WhillNavi2Gui.py
# echo $(ros2 pkg prefix $PACKAGE_NAME)/../../src/$PACKAGE_NAME/doc/GuiScript.sh
python3 $EXEC_PATH