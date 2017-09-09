C.A.V.A. (M.P.)
====================

**C**onsole-based **A**udio **V**isualizer for **A**LSA (**M**PD and **P**ulseaudio)

by [Karl Stavestrand](mailto:karl@stavestrand.no)

![spectrum](https://raw.githubusercontent.com/karlstav/cava/gh-pages/cava_rainbow.gif "spectrum")

# 2017-08-26 - 0.6.0
new features:

* Autosens of low values (dynamic range)
* Removed config parameter 'overshoot'
* Removed config parameter 'mode'
* New config parameter 'waves'
* Changed config parameter 'style' to 'channels' (was either 'mono' or 'stereo' anyway)
* Parameters 'integral' and 'gravity' are now in percentage

# 2017-08-14 - 0.5.0
new features:

* gradient

bugfixes:

* memory leak
* increased config reload speed
* raw output not goint to /dev/stdout

# 2017-04-26 - 0.4.3
new features:

* support for more colors in ncurses output.

bugfixes:

* fixed bug where sensing of low values was done even when no sound
* fixed bug in raw output bit format check
* adde a check for too many bars >200

# 2016-09-24 - 0.4.2
new features:

* raw-output mode
* autosens for low values at start-up
* Reload config file on SIGUSR1
* config option for building with iniparser version 3.1

# 2016-02-10 - 0.4.1
new features:

* added auto pulseaudio source mode
* pulsaudio is now default
* pushing 'r' now reloads audio
* Added console title

bugfixes:

* Fixed error on Raspbian at run-time when running from installed path

considering namechange: cavamp (console based audio visualizer for alsa, mpd and pulseaudio)

# 2015-11-22 - 0.4.0
config-script branch is now master branch

major changes from old master:
* added pulseadio support
* added confg script
* removed commanline argumets
* added optional config path
* added autosens feature
* bugfixes

moved old master in new branch "legacy-0.3.5" just in case.
