AC8 Mouse Aim 0.1.0

Offline single-player only. The mod reads aircraft attitude and replaces only
the three player-control axes. It does not change aircraft physics, thrust,
lift, drag, weapon data, damage, or save files.

Controls
  Mouse      Move the world-space target marker; the aircraft follows it.
  F8         Enable/disable the instructor.
  F9         Recenter the target on the aircraft nose.
  F10        Reload config.ini and recenter.
  RMB        Reserved for game actions; does not release camera or flight control.

The native controller refuses to activate unless the game was started with the
offline Steam launch option, which sets EOS_USE_ANTICHEATCLIENTNULL=1. Never use
this installation for multiplayer: run MouseFlight-Mode.cmd in the game folder to
switch to original mode, and clear the launch option in Steam first.

First flight
  1. Select Expert flight controls in AC8.
  2. Launch from Steam with the offline launch option, then enter a mission.
  3. If pitch or roll moves opposite the cyan marker, close the game, flip the
     matching sign in config.ini, and relaunch. If the two axes are swapped,
     exchange pitch_slot=0 and roll_slot=2.

Logs are written under AC8MouseAim\Logs. Game updates may require a rebuilt
input signature. Remove or disable the mod if AC8 crashes after an update.
