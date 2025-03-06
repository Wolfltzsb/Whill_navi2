SCRIPT_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
# echo $CURRENT_PATH
# echo $SCRIPT_DIR
if [ ! -d $HOME/.local/etc/WhillNavi2Gui ]; then
    mkdir -p $HOME/.local/etc/WhillNavi2Gui
fi

cp $SCRIPT_DIR/GuiScript.bash $HOME/.local/etc/WhillNavi2Gui/GuiScript.bash
cp $SCRIPT_DIR/whill_navi2_gui.desktop $HOME/.local/share/applications/
cp $SCRIPT_DIR/whill_navi2_gui.desktop $HOME/Desktop/
cp $SCRIPT_DIR/face-angry.png $HOME/.local/etc/WhillNavi2Gui/icon.png

if [ ! -d $HOME/.local/etc/WhillNavi2Gui/whill_navi2_source ]; then
    mkdir -p $HOME/.local/etc/WhillNavi2Gui/whill_navi2_source
fi
# echo $SCRIPT_DIR/../../../
ln -sf $SCRIPT_DIR/../../../install/ $HOME/.local/etc/WhillNavi2Gui/whill_navi2_source/
sudo chmod +x $HOME/.local/etc/WhillNavi2Gui/GuiScript.bash
sudo chmod +x $HOME/.local/share/applications/whill_navi2_gui.desktop
sudo chmod +x $HOME/Desktop/whill_navi2_gui.desktop