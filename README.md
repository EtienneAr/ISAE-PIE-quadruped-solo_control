# ISAE-PIE-quadruped-solo_control

# To install libmaster_board_sdk`

`git clone https://github.com/open-dynamic-robot-initiative/master-board.git --reccursive`

Follow the instructions of master-board/sdk/master_board_sdk/example/README.md

When finished, in the build directory type : 

`sudo make install`

`sudo sh -c "echo /usr/local/lib/python3/dist-packages >> /usr/local/lib/python3.5/dist-packages/local.pth"`

**N.B. : change *python3.5* to *python3.6* in the above line depending on your config**

That's it, libmaster_board_sdk is installed and paths are set up
