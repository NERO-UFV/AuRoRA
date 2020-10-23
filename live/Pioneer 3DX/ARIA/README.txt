% ARIA Interfaces for Matlab and Simulink
% September 19, 2013

These interfaces provide a handful of essential features of ARIA in Matlab
or Simulink.

After retrieving this repository, move and rename the resulting directory to
a subdirectory of the Aria installation location: `/usr/local/Aria/matlab` on Linux
or `C:\Program Files\MobileRobots\Aria\matlab` on Windows.


ARIA Simple Interface for Matlab
--------------------------------

A collection of C files (in mex-src) defines MEX interfaces for some of the
functions in ARIA.


## Windows Build ##

Requirements: 

* ARIA 2.7.5.2 or later <http://robots.mobilerobots.com/wiki/ARIA>
* Visual Studio 2010 <http://www.microsoft.com/visualstudio/eng/downloads#d-2010-express>
* Matlab 2012b or 2013a (other versions may work but are untested)
* Windows 7 

Build:

1. This directory should be a subdirectory of ARIA named "matlab" (see above).
2. Build ariac in Release mode using Visual Studio 2010 by opening
   `ariac-vc2010.sln`, choosing Release mode, and building the ariac-dll project.
   *Important:* If you are on a 64-bit platform and are using 64-bit Matlab, choose the x64
   platform.  If you are on a 32-bit platform, use the Win32 target platform.
3. Open Matlab and navigate to this matlab directory.
4. Add this matlab directory to the Matlab path by right-clicking on it in the Matlab file browser and selecting "Add To Path" then "This Folder".
5. Run makemex.m in Matlab to compile the MEX interfaces. It will also copy
       `AriaVC10.dll` from bin or bin64 into the matlab directory.
 
 
The MEX interfaces require `ariac.dll` and `AriaVC10.dll` to be present in the matlab directory to 
build and run.
 
To run scripts/programs in Matlab, the matlab directory (containing the compiled
MEX objects as well as ariac.dll and AriaVC10.dll) must be in your Matlab path. It can be added
by right-clicking on the matlab directory in the file browser in Matlab and selecting "Add To Path", then "This Folder".

Or use the path command: 

    path(path, 'C:\Program Files\MobileRobots\Aria\matlab')

Or if your current directory is the Aria matlab subdirectory: 

    path(path, '.')  

## Linux Build ##

Requirements:

* ARIA 2.7.5.2 or later <http://robots.mobilerobots.com/wiki/ARIA>
* G++ compiler and GNU make installed (standard Linux development tools). The compiler version must
match supported MEX compiler (see below).
* Matlab 2012a or later

Build:

1. This directory should be a subdirectory of ARIA named `matlab` (see above).
2. Enter the matlab directory and run `make`. This will build the `libariac.so` C 
library in `/usr/local/Aria/lib`, and will run Matlab commands in `makemex.m` to 
build the MEX interface.  You can also run `makemex` within Matlab. If the 
`matlab` command is not in your PATH, you can add it with this shell command:
`export PATH=$PATH:/usr/local/MATLAB/R2012b/bin`
Substitute the correct installation location and version for your Matlab installation.
If any warnings are printed regarding unsupported compiler versions, you may need
to switch to a supported compiler as reported by Matlab.  You can do so by 
installing the correct compiler, setting the CC and CXX environment variables
to that compiler command, and rebuilding ARIA and ariac (enter `/usr/local/Aria`,
run `make clean`, then enter the `matlab` directory, and re-run `make`.
3. Run `ldconfig` as root, so that the new `libariac.so` library is usable.

The ARIA matlab directory (containing the compiled MEX interfaces) must be in your Matlab path.
It can be added by right-clicking on the Aria matlab directory in the GUI and selecting
"Add To Path" then "This Folder", or by using the path command:

    path(path, '/usr/local/Aria/matlab')

If you wish to always add the Aria matlab directory to your Matlab path, you can do so in
one of two ways:

You an append `/usr/local/Aria/matlab` to the `MATLABPATH` environment variable in your
`.matlab7rc.sh` script in your home directory if you wish to permanently add 
the Aria matlab directory to your Matlab path:

    export MATLABPATH=$MATLABPATH:/usr/local/Aria/matlab

(See <http://www.mathworks.com/help/matlab/ref/matlabunix.html> for more about the `.matlab7rc.sh` script)

Or, you can create a `startup.m` script in a directory where you will keep your Matlab
scripts that use the Aria interface.  If you run the `matlab` command from within
this directory, then Matlab will automatically run `startup.m`.  Your `startup.m`
can use the `path` function to add the Aria matlab directory to the path, and execute
any other commands you wish.

    path(path, '/usr/local/Aria/matlab')

## Usage ##

See `example.m` for a simple example of use.
 
So far, the following functions are available:

`aria_init` [args]

Initialize ARIA and store any ARIA arguments given. This function must be called
first before any other of the following functions can be used. 

`arrobot_connect`

Connect to the robot and begin background communications/processing cycle thread.

`arrobot_disconnect`

Disconnect from the robot if connected and free/destory internally used memory/objects.

`[x, y, th] = arrobot_getpose`

`x = arrobot_get`

`y = arrobot_gety`

`t = arrobot_getth`

Get current position estimate from the robot

`arrobot_setvel(v)`

Set translational velocity

`arrobot_setrotvel(v)`

Set rotational velocity

`arrobot_setvels(x, y)`

`arrobot_setvels(x, y, th)`

`arrobot_setvels [x, y, th]`

Set all velocity components. Note, only Seekur can perform Y (lateral) velocity.

`arrobot_stop`

Stop the robot

`v = arrobot_getbatteryvoltage`

Get battery voltage (real voltage, e.g. 0..12 for 12V system, 0..24 for 24V systems, etc.)

`s = arrobot_isstalled`

`s = arrobot_isrightstalled`

`s = arrobot_isleftstalled`


Return 1 if either wheel motor is in stalled state

`e = arrobot_motorsenabled`

Return 1 if the motors are enabled, or 0 if disabled (by disable command or e-stop button)

`arrobot_enable_motors`

`arrobot_disable_motors`

Enable or disable the motors. Use `arrobot_enable_motors` to manually re-enable the motors if disabled
due to e-stop button or other event.

`l = arrobot_length`

`w = arrobot_width`

`r = arrobot_radius`

Size of the robot

`n = arrobot_getnumsonar`

Number of sonar range readings

`r = arrobot_getsonarrange(i)`

Get range (mm) of sonar sensor i. 

`[s1, s2, s3... sN] = arrobot_getsonar`

Get ranges (mm) of all sonar sensors

`arrobot_setdigout(d)`

Set state of digital output bits according to bitmask (8 bits)

`d = arrobot_getdigin`

Get state (bitmask) of digital input bits (8 bits)

`arrobot_command(c, [args])`

Send any controller command to the robot (see robot manual). [args] can be one
integer argument or two integers (for commands that take two byte arguments)

`arloginfo`

Log some internal debugging information about Aria, the matlab interface and ariac.

See documentation in `ariac.h` for more information on the functions.

Currently the Aria Matlab interface can only connect to one robot, and can
only connect directly to the robot via serial connection (by running
on an onboard computer or computer directly connected), or a TCP
connection to a robot's wifi interface device.


How to add new functions to the Matlab interface
------------------------------------------------

The Aria-Matlab interface is intended to be a base upon which any additional features 
and functions you require can be easily added.

To add a function to the Matlab interface that is not currently available,
it must first be added to the ariac C library (if not already present in ariac).  
Any objects needed for the new feature must be instantiated in `arrobot_connect` 
or similar, and a pointer stored in ariac (see how the ArRobot object and others are currently 
created and stored). This object can be destroyed in the disconnect/exit 
functions.  Generally, ariac should be robust against functions called more than 
once, so check if the object has already been created before creating it a second time. 
To avoid problems using Simulink, it is also recommended that functions check for 
robot connection and other neccesary preconditions, and abort (return) if not able to continue
rather than crashing.  (This allows some Simulink simulations to begin running even before
the robot connection has finished.)

If you have called any of the Aria functions in your currently running Matlab
instance, run `clear all` in Matlab to unload the functions and the ariac
library/DLL.

Then recompile ariac (using Visual C++ on Windows or make on Linux). 

Next, a `mexFunction` must be created in a new source file in the `mex-src` directory
that implements an interface between Matlab and the new function in ariac.
The new source file in the mex-src directory determines the name of the function
in Matlab, and should match the name of the new function in ariac.  This source
file must contain a function called mexFunction which receives Matlab data structures
containing any arguments given with the Matlab function call, and into which return
values can be placed. See the existing functions in mex-src for examples, and the
Matlab documentation for mexFunction (`doc mexFunction`).

Finally, add the new function to the list of functions in `makemex.m`, and re-run
`makemex.m` in Matlab.





ARIA Simple Interface for Simulink
----------------------------------

To use the Simulink interface, first follow the steps above to build ariac
and the Matlab interface. A compiler must have been selected first by running 
`mex -setup` in the Matlab command window (this only needs to be done once).  
On Windows, choose Visual C++ 2010.  Build ARIA and ariac with Visual C++ 2010 
in "Release" mode.

The next step is to create the Simulink S-function blocks that will interact with the ariac library to communicate with the robot. These blocks are created by running the `sdefs.m` script within the `matlab` folder.  This script uses the "legacy code tool"  to create Simulink blocks.  You can do this by navigating to this matlab 
folder in the file browser, and then either  running `sdefs` in the command window, 
right clicking on `sdefs.m` in the file browser and choosing Run, or opening `sdefs.m`
in the text editor and clicking "Run" in the toolbar.  For each function, the 
sdefs.m script will generate an s-function wrapper and compile it, and generate a 
simulink block in the ariac Simulink library (`ariac-simulink-library/ariac.mlx`),
and refresh Simulink's block library browser.  

You can add simulink-library to your Matlab path for 
the  block library to appear in the Simulink block library whenever you 
run Matlab. Do this by right clicking on `ariac-simulink-library` in the Matlab
file browser, choosing "Add To Path", and choosing "This Folder".

Simulink Tips
-------------

* Open the Model Configuration Parameters (the "gear" or "cog" icon) to set
  the solver to appropriate behavior such as "Fixed-step", possible use the discrete
  solver engine if desired,  set the fixed step size, and increase the simulation
  end time (to let the simulation run for long enough to perform your tests and expermients.)
* Run `aria_init` and `arrobot_connnect` in the Matlab command window before runnning
  your Simulink model.  Or call them automatically from model callbacks (see next item below).
* You can set model callbacks to perform actions such as automatically initalize
  and connect to the robot, stop robot motion,
  reset odometry, etc. when the Simulink model is initialized, stopped, etc. To set model 
  callbacks, use the drop down menu attached to the "Model Configuration"
  button in the toolbar in the model editor (the "gear" or "cog" icon) and choose 
  "Model Properties", then select the "Callbacks" tab, then enter Matlab code
  to run on each of the conditions.   For example, enter `arrobot_stop` for `StopFcn`
  to automatically stop robot motion if you stop the Simulink simulation. More information is at
  <http://www.mathworks.com/help/simulink/ug/model-callbacks.html>
* Make sure the ARIA matlab directory and `ariac-simulink-library` directory are in
  your Matlab path in order to see the AriaC blocks in the Simulink library browser,
  and for blocks to appear in your model. If ariac blocks are missing from your model,
  re-add the matlab and `ariac-simulink-library` directories to your path, then right
  click and choose Refresh.   You can also regenerate the s-function ariac blocks
  by running `sdefs.m` (see previous section).


How to add new functions to the Simulink interface
--------------------------------------------------

To add a feature to the Matlab interface that is not currently available,
it must first be added to the ariac C library.  Any objects needed for the
new feature must be instantiated in `arrobot_connect` or similar, and a 
pointer stored in ariac (see howArRobot etc. are currently created and stored).
This object can be destroyed in the disconnect/exit functions.  Generally,
ariac should be robust against functions called more than once, so check if 
the object has already been created before creating it a second time.  

If you have called any of the Aria functions in your currently running Matlab
instance, run `clear all` in Matlab to unload the functions and the ariac
library/DLL.

Recompile ariac.

Next, add the function to the list in `sdefs.m`, and re-run `sdefs.m` to 
regenerate all the S-functions.

Simulink External Mode
----------------------

If you have the optional Simulink Coder component, you can compile a Simulink system 
and deploy it to  the robot's onboard computer via SSH, while viewing and starting/stopping
system from the Simulink GUI on a remote laptop or other computer over wifi.  

For details, see: <file:Simulink External Mode.docx>.

External Mode is only supported on Windows operating systems at this time.


About ariac
-----------

ariac is a simple functional C wrapper to ARIA that provides the minimum API for connecting to 
a robot, receiving data from it, and controlling it by requesting velocities. All
neccesary ARIA objects and other state are stored internally by ariac, so the
caller (e.g. Matlab) does not need to store and pass in any object references,
handles, identifiers or any other state -- only call functions.

A Makefile is included which can build just this simple C interface as a library
called libariac on Linux.  ariac links dynamically to ARIA.  Any program linking 
to it must also link to ARIA and dependent libraries:

On Linux, link to libAria, libdl, librt, libm, e.g.:
    gcc -fPIC -o myprog myprog.c -L/usr/local/Aria/lib -lariac -lAria -lpthread -ldl -lrt -lm

On Windows, link to `Aria` or `AriaDebug`, `ws2_32` and `winmm`, and include the ARIA
lib directory in the linker path. (Include the ARIA bin directory in system path
or copy DLLs when running the program.)

See `ariac-test.c` for a simple example of how ariac is used.

Currently ariac asynchronously accesses the ArRobot object, using ArRobot's 
`lock()` and `unlock()` functions to prevent incorrect simultaneous access by other
threads such as ArRobot's processing cycle background thread. This can be changed
in the future if neccesary to improve performance.   However, this will not affect
ariac's public API, just its internal behavior and performance charactaristics.


Acknowlegements
---------------

Special thanks to Mathworks for their support in developing these interfaces, especially Fred Banser for his work developing the Simulink interface and ability to deploy and run via external mode.

