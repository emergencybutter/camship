FLYCAM
======

An X-Plane 11.50+ / 12 plugin that films your aircraft like a camera plane
shooting an ad. It cuts between moving outside shots: formation slides,
overtakes, orbits, head-on and top-down views, a wingtip camera, engine
close-ups and flybys.

Works on Windows, macOS (Intel and Apple Silicon) and Linux.


INSTALLATION
------------

1. Quit X-Plane.
2. Copy the whole "FlyCam" folder from this zip into
   <X-Plane folder>/Resources/plugins/
   You should end up with:
       Resources/plugins/FlyCam/64/win.xpl
       Resources/plugins/FlyCam/64/mac.xpl
       Resources/plugins/FlyCam/64/lin.xpl
3. Start X-Plane. "FlyCam" now appears in the Plugins menu.

Updating (recommended): use SkunkCrafts Updater. It picks up the plugin
from the included skunkcrafts_updater.cfg and downloads new versions for
you, so you only ever install this zip once.

Updating by hand: quit X-Plane and replace the FlyCam folder. Your
settings are kept either way.

macOS: if X-Plane refuses to load the plugin because it was downloaded,
run this in Terminal (adjust the path to your X-Plane folder):
    xattr -dr com.apple.quarantine "/path/to/X-Plane 12/Resources/plugins/FlyCam"


BASIC USAGE
-----------

1. Plugins > FlyCam > Cinematic camera starts the camera. Select it again
   to return to your previous view. Picking any other X-Plane view also
   stops it.
2. Plugins > FlyCam > Settings... opens the FlyCam window:
     - Start/Stop, Next shot, and Hold (keep the current shot going).
     - Tick the shots you want. The play button next to a shot cuts to it
       straight away, so you can preview it.
     - Camera: shot length, distance, zoom, turn lag, and cut or glide
       between shots.
   Changes apply immediately and are saved by themselves.

Tips:

  - Bind keys or joystick buttons in Settings > Keyboard / Joystick to:
        flycam/toggle
        flycam/next_shot
        flycam/hold
        flycam/toggle_window
  - X-Plane screenshots (Shift+Space) and movie recording capture
    FlyCam's view.
  - Engine close-ups need the aircraft to report its engine positions;
    they are greyed out in the window when it doesn't.
  - Settings are saved in X-Plane/Output/preferences/flycam.prf


TROUBLESHOOTING
---------------

Look for lines starting with "FlyCam:" in X-Plane/Log.txt. They show the
aircraft size and engine positions FlyCam found. If shots are framed too
close or too far for an aircraft, set subject_size_m (its wingspan in
meters) in flycam.prf while X-Plane is not running.


SOURCE CODE
-----------

https://github.com/emergencybutter/flycam


LICENSE
-------

MIT. See LICENSE.txt.
