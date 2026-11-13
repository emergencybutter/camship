CAMSHIP FOR MSFS 2024
=====================

A small Windows app that films your aircraft in Microsoft Flight Simulator
2024 like a camera plane shooting an ad. It cuts between moving outside
shots: formation slides, overtakes, orbits, head-on and top-down views, a
wingtip camera and flybys. When something happens (gear, flaps, touchdown,
rotation, spoilers and more) it cuts to a shot made for that moment.

It is the same camera director as the Camship X-Plane plugin, driving
MSFS 2024's SimConnect camera API. That API is still marked beta by
Microsoft/Asobo, so this version is experimental.


INSTALLATION
------------

1. Copy the "Camship-MSFS" folder anywhere, e.g. your Documents.
2. Start MSFS 2024 and load a flight.
3. Run Camship.exe. Its window says "Connected to MSFS" once it has found
   the simulator. You can start Camship before MSFS: it waits for it.

No SDK, SimConnect.dll or installer is needed.


BASIC USAGE
-----------

  - Start (or Next shot) takes MSFS's camera and starts filming. Stop gives
    it back. Changing the view in MSFS also stops it.
  - Tick the shots you want. The play button next to a shot cuts to it
    straight away, so you can preview it.
  - Camera: shot length, distance, zoom, turn lag, and the pause, and
    whether to keep filming while the sim is paused.
  - "Log" at the top shows what Camship is doing.

Changes apply immediately and are saved by themselves.


WHAT'S DIFFERENT FROM THE X-PLANE VERSION
-----------------------------------------

  - MSFS doesn't tell add-ons where the engines and wheels are, so engine
    close-ups are off and gear shots use a typical layout.
  - MSFS only reports the ground height under the aircraft, so the camera's
    ground clearance assumes flat ground around it. Camship also turns on
    MSFS's own "keep the camera above ground" option.
  - No key bindings yet: use the window.


SETTINGS FILES
--------------

Both live in %APPDATA%\Camship (paste that into Explorer's address bar):

  camship.prf        shots and camera settings (same format as X-Plane's)
  camship-msfs.ini   connection and camera calibration, with comments
  Log.txt            what happened, for troubleshooting

If the camera points the wrong way, is upside down or zoomed oddly, the
calibration keys in camship-msfs.ini (pitch_sign, bank_sign, fov_units,
position) let you fix it without a new version. Please report what worked.

To use a simulator on another PC, set "address host:port" in
camship-msfs.ini (MSFS needs an IPv4 entry in its SimConnect.xml), or run
    Camship.exe --address 192.168.1.20:500


SOURCE CODE
-----------

https://github.com/emergencybutter/camship


LICENSE
-------

MIT. See LICENSE.txt.
