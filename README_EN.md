# AC8 MouseFlight

English | [简体中文](README.md)

War Thunder-style mouse aim for **ACE COMBAT 8 offline single-player**: point the aim with the mouse and the aircraft brings its nose there by rolling its lift onto the aim and pulling.

> Offline single-player only. Multiplayer is not supported, and compatibility with other input, camera or loader mods is not guaranteed. A game update may stop an older version of the mod from working.
> This is an unofficial mod, not affiliated with the game's developer or publisher, or with War Thunder.

## Overview

The mod replaces only the player's control inputs and corrects the camera; it does not change the flight model, thrust, weapons or save data.

- **Mouse aim**: the aim is a direction in the world. The flight control rolls the lift onto it, then pulls, stopping and leveling on a measured model of the aircraft. There are no ground, stall or g limits: the player's aim is followed as it is.
- **Camera**: the chase camera turns smoothly to the aim. The cockpit and nose views keep the game's camera, with a mouse stick and an optional head turn toward the ring. The field of view (FOV) can be offset.
- **Working with the game**: control and camera are handed back during the lock-on view, cutscenes and the autopilot. Manual takeover follows the game's own key bindings.
- **Settings panel**: change the common settings and every key in game, applied at once. Its language follows the game's (English, Simplified Chinese).

## Status

- Current version: `0.2.30+Alf.1.0.0`. Before the `+` is the upstream release this fork is based on; after `Alf.` is this project's own version. See the [changelog](CHANGELOG.md) (in Chinese).
- Target: Windows x64, ACE COMBAT 8 Steam build 25201480. The native module checks the game's and the loader's signatures and refuses to run on a mismatch.
- In-game testing has been done on the developer's machine only. See the [current status](docs/00-overview/current-status.md) and the [capability matrix](docs/01-capabilities/capability-matrix.md) (in Chinese).

## Flight control comparison

[Flight Bench](https://kitasanyuu.github.io/AC8-Mouse-Aim/): this project's flight control, upstream FletcherMiya 0.2.30 and xsd467 pw.11, each flying the same scenes (general maneuvers, landings, a multi-aircraft fight, boss fights) on the same aircraft model with the same simulated pilot, compared scene by scene (aim error, kills, overall cost) with a 3D replay. These are simulation results, not the in-game experience; the method and its limits are in the [flight bench](docs/04-development/flight-bench.md) document (in Chinese). The page is in Chinese.

## Quick start

1. Quit the game completely, then download (or `git clone`) this repository.
2. Run `AC8MouseFlight.cmd` and choose **1** to install.
3. After the first install, set the Steam launch options (they are copied to the clipboard at the end of the install):

   ```text
   cmd /d /c "set EOS_USE_ANTICHEATCLIENTNULL=1&& %command% -anticheat_settings=AC8MouseAim_Offline.json"
   ```

4. Start the game, choose the EXPERT control type and enter a single-player mission. Borderless windowed mode is recommended.

| Default key | Function |
|---|---|
| Mouse | Move the aim |
| F (hold) | Free look |
| F3 | Settings panel |
| F8 | Mouse aim on/off |
| F9 | Aim back on the nose |

All keys can be changed on the panel's Key Config page. **Before playing multiplayer, switch to the online (vanilla) mode with `AC8MouseFlight.cmd` and clear the launch options.**

## Documentation

The documentation is in Chinese: start from the [documentation index](docs/README.md). Build instructions are in [build and deploy](docs/04-development/build-and-deploy.md).

## Credits and license

This project is a fork of [FletcherMiya/AC8-Mouse-Aim](https://github.com/FletcherMiya/AC8-Mouse-Aim) by Fletcher Gong (CC0 1.0). The original repository laid the mod's foundation: the UE4SS loading and offline launch, the native input hook, the camera correction and the overlay. This project rewrote the flight control on top of it and added the cockpit and nose views, the settings panel and localization, among others.

- [xsd467/AC8-Mouse-Aim](https://github.com/xsd467/AC8-Mouse-Aim): the pw.11 (05d7f48) and pw5 flight controls, CC0 1.0, included as reference controllers in the flight bench.
- [RE-UE4SS](https://github.com/UE4SS-RE/RE-UE4SS): mod loader, Lua runtime and interfaces, MIT license.
- [MinHook](https://github.com/TsudaKageyu/minhook): native function hooks, BSD 2-Clause license.

The project's own code is licensed as stated in `LICENSE.txt`; third-party components in `THIRD_PARTY_NOTICES.txt`. The project contains no game assets or game executables.
