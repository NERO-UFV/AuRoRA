# Range Finder Deterministic Object Detection - Simulation

This repo code and scene for simulation of deterministic object detection in a V-REP simulation using the AuRoRA data structure and framework. 

## Running this simulation
- [Download CoppeliaSim V-REP Education v4.1.0](https://www.coppeliarobotics.com/downloads.html)
- After installed, inside V-REP, open **YourCustomPath...**```\AuRoRA\live\V-REP Simulations\Object Detection\cena_laser_clutter.ttt``` scene
- Press the purple **PLAY** button (*"Start/resume simulation"*)
- In MATLAB, click **Set Path** near _Layout_ under the Home tab and click _Add with subfolders_, then add the whole AuRoRA folder
- Open ```tim_laser_completo.m``` in MATLAB and run! :wink:

## Known issues
- For some reason, the V-REP pioneer starts slowly moving (maybe small errors being summed up over time?) and gets out of the starting position. To reset, press the purple square **STOP** button then press the purple **PLAY** again.
