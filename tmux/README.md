# Setting up tmux to facilitate work on Game Engine

tmux is a handy terminal multiplexer that you may find useful as you work
within Game Engine.  It allows you to control multiple shells in the same
terminal, which is convenient when you have several different executables
(e.g., `mediation_layer`, `student_autonomy_protocol`, etc) that you need to
run at the same time.  tmux is already installed on the VM.  You can use it in
its default mode without any further setup.  If you'd like to adopt
Dr. Humphreys's setup, follow the instructions below.

## Setup

First clone and install the RNL public `dotfiles` repository so tmux and vim
will be configured to operate according to the RNL style:
```bash
cd ~
git clone https://gitlab.com/radionavlab/public/dotfiles.git
cd dotfiles
./install
```

Next, move the `stmux` bash script into `/usr/local/bin` so that it can be run
by typing `stmux` on the command line:
```bash
cd ~/Workspace/game-engine/tmux
sudo cp stmux /usr/local/bin
```

## Running tmux

You'll find
[here](https://www.hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/) a good
tutorial on tmux. Note that the tmux configuration you installed from
`dotfiles` alters the default tmux behavior somewhat:

- By default, tmux's prefix key is `C-b` (Ctrl-b).  But this requires pressing
  two buttons!  The RNL's prefix key is the backtick `, which is right next to
  the numeric 1 key.  You're welcome to edit `~/.tmux.conf` if you'd like to
  change back to the original prefix, or select another prefix.
- To switch to window 2: prefix 2
- To split the current pane vertically: prefix Left or Right
- To split the current pane horizontally: prefix Down or Up
- To navigate between panes: C-Down or Up or Right or Left
- To close a pane: C-d

You can have a look at all tmux settings by inspecting the file
`~/.tmux.conf`.

To set up a tmux session tailored for working in Game Engine, open a new
terminal and type `stmux`.

