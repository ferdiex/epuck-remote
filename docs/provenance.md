# Provenance

## Code copied from earlier projects (author's own)
- `sim/sensors.py`: the e-puck part of `easabrain_sensors.py`, verbatim;
  the Khepera II section removed. easabrain took it from easagru
  (`easagru_sensors.py`, commit 2c3191d).
- `sim/world.py`: the e-puck functions of `easabrain_world.py`, verbatim;
  Khepera loading removed, paths resolved from the repo root.
- `sim/config.json`: the world, e-puck, sensors, floor and food values of
  `easabrain_config.json` (from easagru, commit 2c3191d).
- `robots/make_epuck_urdf.py`, `robots/epuck.urdf`, `robots/epuck_lean.urdf`:
  copied unchanged from easabrain.
- `controllers/braitenberg_avoidance.json`: from the essim project.

## External sources
- e-puck model measurements: Webots e-puck PROTO files by Cyberbotics,
  https://github.com/cyberbotics/webots/tree/released/projects/robots/gctronic/e-puck/protos
  (E-puck.proto, E-puckDistanceSensor.proto, E-puckGroundSensors.proto).
  Not redistributed here: their licence allows use with Webots only.
- asercom protocol: GCtronic wiki, "Advanced sercom protocol",
  https://www.gctronic.com/doc/index.php/Advanced_sercom_protocol.
  Implemented from the documentation; no GCtronic code is included.
