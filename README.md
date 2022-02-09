# dome
Geodesic Dome Math

## Python dome tool

[dome.py](dome.py) calculates an icosahedron or 2v geodesic dome and creates 3d printable hubs as [OpenSCAD](https://openscad.org/) sources.

```
usage: dome.py [-h] [--hub HUB] [--ico] [--strut-dia STRUT_DIA] [--socket-length SOCKET_LENGTH] [--core-hole-dia CORE_HOLE_DIA] [--thick THICK] [--outside-extra OUTSIDE_EXTRA] [--flip] [--report]
               [--stock-length STOCK_LENGTH] [--rings RINGS] [-v] [--debug-spheres]

optional arguments:
  -h, --help            show this help message and exit
  --hub HUB             single hub to generate scad for
  --ico                 icosahedron instead of 2v geodesic
  --strut-dia STRUT_DIA
                        strut diameter, default 25.4/2mm, may specify "XXin" inches
  --socket-length SOCKET_LENGTH
                        socket interior length, default (strut-dia * 2)
  --core-hole-dia CORE_HOLE_DIA
                        core hole dia, in or mm
  --thick THICK         socket wall thickness, in or mm
  --outside-extra OUTSIDE_EXTRA
                        extra height on outside of hub
  --flip
  --report
  --stock-length STOCK_LENGTH
                        how long does strut stock come in, ft/in/mm
  --rings RINGS         how many rings of a geodesic dome do you want rendered (1 = just the top pentagon)
  -v, --verbose
  --debug-spheres
```

## Javascript + three.js render

[js/dome.js](js/dome.js) uses `three.js` to render a 2v geodesic dome and do various calculations on the points and edges.
