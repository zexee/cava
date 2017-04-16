C.A.V.A. (M.P.)
====================

**C**onsole-based **A**udio **V**isualizer for **A**LSA (**M**PD and **P**ulseaudio)

by [Karl Stavestrand](mailto:karl@stavestrand.no)

![spectrum](https://cloud.githubusercontent.com/assets/5231746/7368413/7d7ca93e-ed9f-11e4-842f-61c54fc03dbe.gif "spectrum")

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