#!/usr/bin/python3

import io
import logging
import math

import numpy
import numpy.linalg
from scipy.spatial.transform import Rotation

logger = logging.getLogger(__name__)

# center = 0,0,0
# r = 1
# icosahedron base case with subdivision

# TODO: emit STL ?

# https://en.wikipedia.org/wiki/Golden_ratio
phi = (1 + math.sqrt(5)) / 2

def vlen(they):
    return math.sqrt(sum([x*x for x in they]))

def dist(a,b):
    m = 0
    for va,vb in zip(a,b):
        d = va-vb
        m += d*d
    return math.sqrt(m)

def feq(a,b,ep=0.0000001):
    return abs(a-b)<ep

def norm(pt):
    r = vlen(pt)
    return tuple(x/r for x in pt)

def mid(a, b):
    return tuple((va+vb)/2 for va,vb in zip(a,b))

def dot(a, b):
    out = 0
    for av, bv in zip(a,b):
        out += av * bv
    return out

def feq(a, b):
    return abs(a - b) < 0.000001

# https://en.wikipedia.org/wiki/Regular_icosahedron#Cartesian_coordinates
icoCartesianPoints = [
    (0,phi,1),
    (1,0,phi),
    (phi,1,0),

    (0,-phi,1),
    (1,0,-phi),
    (-phi,1,0),

    (0,phi,-1),
    (-1,0,phi),
    (phi,-1,0),

    (0,-phi,-1),
    (-1,0,-phi),
    (-phi,-1,0),
]

def nearestNeighbors(points):
    neighbors = []
    # find each point's nearest other points
    for i, pt in enumerate(points):
        neighbors.append([])
        mindist = None
        ties = None
        for j, xpt in enumerate(points):
            if j == i:
                continue
            d = dist(pt, xpt)
            if (mindist is None) or (d < mindist):
                mindist = d
                ties = [j]
            elif feq(d, mindist):
                ties.append(j)
        neighbors[i] = ties
    return neighbors

# for radius to vertices, edge length 'a'
# r = (a / 2) * sqrt(phi * sqrt(5))
# edge_length = 2 * r / sqrt(phi * sqrt(5))

def binfit(stockLen, kerf, endTrim, lencounts, r, out):
    # lencounts should have been sorted, longest first
    piececount = 0
    return piececount

class Wireframe:
    def __init__(self, points, edges):
        self.points = points # [(x,y,z), ...]
        self.edges = edges # [(i,j), ...] incices into points
        self._endTrim = 0
    def edgeLengths(self):
        # lencounts = [[length, count], ...]
        lencounts = []
        for i,j in self.edges:
            el = dist(self.points[i], self.points[j])
            fit = False
            for lcp in lencounts:
                xl = lcp[0]
                if feq(xl, el):
                    # maintain running average
                    lcp[0] = ((xl * lcp[1]) + el) / (lcp[1] + 1)
                    lcp[1] += 1
                    fit = True
                    break
            if not fit:
                lencounts.append([el, 1])
        return lencounts
    def stockHack(self, stockLen, kerf=1.0):
        "calculate cuts given length of stock (mm), kerf is amount lost as sawdust"
        lencounts = sorted(self.edgeLengths(), reverse=True)
        out = io.StringIO()
        # dummy solution, longest member == whole stock length
        piececount = lencounts[0][1]
        r = (stockLen + (2*self._endTrim)) / lencounts[0][0]
        out.write('r={}\n'.format(r))
        out.write('{} whole pieces as {}*r length member\n'.format(lencounts[0][1], lencounts[0][0]))
        if len(lencounts) > 1:
            piececount += binfit(stockLen, kerf, self._endTrim, lencounts[1:], r, out)
        return out.getvalue()
    def pointsOpenSCAD(self, r=0.1):
        out = io.StringIO()
        for pt in self.points:
            out.write('translate([{}, {}, {}]) sphere(r={});\n'.format(pt[0], pt[1], pt[2], r))
        return out.getvalue()

    def hubOpenSCAD(self, hi, strutDia=25.4/2, thick=None, socketLength=None, coreHoleDia=25.4/4, outsideExtra=0, flip=False, debugSpheres=False):
        logger.debug('hubOpenSCAD(%d, strutDia=%r, thick=%r, socketLength=%r, coreHoleDia=%r)', hi, strutDia, thick, socketLength, coreHoleDia)
        eset = set()
        for i,se in enumerate([tuple(sorted(e)) for e in self.edges]):
            if se in eset:
                logger.error('edge[%d] dup %r', i, se)
            eset.add(se)
        if socketLength is None:
            socketLength = strutDia * 2
        if thick is None:
            thick = strutDia / 7
        logger.debug('hubOpenSCAD(%d, strutDia=%r, thick=%r, socketLength=%r, coreHoleDia=%r)', hi, strutDia, thick, socketLength, coreHoleDia)
        otherpi = []
        for ia, ib in self.edges:
            #logger.debug('edge (%d %d)', ia, ib)
            if ia == hi:
                otherpi.append(ib)
            elif ib == hi:
                otherpi.append(ia)
        logger.debug('pt[%d] at %r neighbors pts %r', hi, self.points[hi], otherpi)
        otherpts = [self.points[x] for x in otherpi]
        otherpts = numpy.array(otherpts)
        hpt = numpy.array(self.points[hi])
        #logger.debug('otherpts %r', otherpts)
        print(otherpts)
        if hi != 0:
            # rotate points so that selected point is straight at [0,0,-1]
            az = math.atan2(hpt[1], hpt[0])
            xy = vlen(hpt[:2])
            alt = math.atan2(hpt[2], xy)
            rotr = (math.pi/2) + alt
            naz = az + math.pi/2
            rot = Rotation.from_rotvec([rotr * math.cos(naz), rotr*math.sin(naz), 0])
            # flip
            # rot = rot * Rotation.from_rotvec([math.pi,0,0])
        else:
            az = 0
            alt = math.pi/2
            # flip
            rot = Rotation.from_rotvec([math.pi,0,0])
        ntop = rot.apply(hpt)
        logger.debug('%r (az %f, alt %f) -> %r', hpt, az, alt, ntop)
        assert(abs(ntop[0]) < 0.0001)
        assert(abs(ntop[1]) < 0.0001)
        otherpts = rot.apply(otherpts)
        print(otherpts)
        hpt = numpy.array([0,0,-1])
        # endTrim is the amount to trim off eatch end of a strut vs ideal point to point length
        # "* math.pi / 2" is not perfectly principled; the ideal would be based on the diameter of the strut and the angles involved such that there would be just a little space between the bases of each strut in the hub; but this is easy and close enough.
        endTrim = strutDia * math.pi / 2;
        self._endTrim = endTrim
        out = io.StringIO()
        out.write('''module hub{}() {{
strutDia = {};
thick = {};
socketLength = {};
endTrim = {};
outsideExtra = {};
'''.format(hi, strutDia, thick, socketLength, endTrim, outsideExtra))
        shells = []
        bores = []
        minz = min([pt[2] for pt in otherpts])
        #logger.debug('minz %f', minz)
        coreR = 0
        coreTop = 0
        maxStrutY = 0
        for pt in otherpts:
            dpt = pt - hpt
            az = math.atan2(dpt[1], dpt[0])
            walt = math.atan2(dpt[2], vlen(dpt[:2]))
            wdy = math.sin(walt-(math.pi/2))*(strutDia/2+(thick*0.8))
            socketEndCenterY = math.sin(walt)*(socketLength+endTrim+thick)
            maxStrutY = max(maxStrutY, socketEndCenterY-wdy)
        dbspheres = []
        for pt, opi in zip(otherpts, otherpi):
            dpt = pt - hpt
            az = math.atan2(dpt[1], dpt[0])
            alt = math.atan2(dpt[2], vlen(dpt[:2]))
            # rotate down to alt from pointing straight up
            rotd = 90 - (alt * 180 / math.pi)
            naz = az + math.pi/2
            #naz = az
            logger.debug('az %f, alt %f', az, alt)
            rotate = 'rotate({}, [{}, {}, {}])'.format(rotd, math.cos(naz), math.sin(naz), 0)
            shells.append('{} translate([0,0,endTrim-thick]) cylinder(r=(strutDia/2)+thick, h=socketLength+thick);'.format(rotate))
            bores.append('{} translate([0,0,endTrim]) cylinder(r=strutDia/2, h=socketLength+1);'.format(rotate))
            bores.append('rotate({}, [0,0,1]) translate([endTrim*.8,0,coreThick-outsideExtra-0.5]) linear_extrude(1) text("{}", size=endTrim/6, valign="center", halign="center");'.format(az*180/math.pi, opi))

            # wedge
            walt = alt
            socketEndCenterX = math.cos(walt)*(socketLength+endTrim)
            socketEndCenterY = math.sin(walt)*(socketLength+endTrim)
            wdx = math.cos(walt-(math.pi/2))*(strutDia/2+(thick*0.8))
            wdy = math.sin(walt-(math.pi/2))*(strutDia/2+(thick*0.8))
            #logger.debug('socket end %f (%f,%f) wd %f (%f,%f)', walt, socketEndCenterX, socketEndCenterY, alt, wdx, wdy)
            shells.append('rotate({}, [0,0,1]) rotate(90,[1,0,0]) translate([0,0,-thick]) linear_extrude(thick*2) polygon([[0,0], [endTrim,0], [{},{}], [{},{}], [0,coreThick]]);'.format(az*180/math.pi, socketEndCenterX+wdx, socketEndCenterY+wdy, socketEndCenterX-wdx, maxStrutY))

            # debug spheres
            if debugSpheres:
                dbpt = dpt * (1/numpy.linalg.norm(dpt))
                logger.debug('pt %r - hpt %r => dpt %r', pt, hpt, dpt)
                dbspheres.append('translate([{}, {}, {}]) sphere(r=strutDia/2);'.format(*(dbpt*(thick+socketLength+endTrim)*1.3)))
        out.write('coreThick = {};\n'.format(maxStrutY+outsideExtra))
        if coreHoleDia is not None:
            bores.append('translate([0,0,-1-outsideExtra]) cylinder(r={}, h=coreThick+outsideExtra+2);'.format(coreHoleDia/2))
        bores.append('rotate(30, [0,0,1]) translate([endTrim/2,0,coreThick-outsideExtra-0.5]) linear_extrude(1) text("{}", size=endTrim/4, valign="center", halign="center");'.format(hi))
        bores.append('rotate(180,[1,0,0]) translate([endTrim/2,0,outsideExtra-0.5]) linear_extrude(1) text("{}", size=endTrim/4, valign="center", halign="center");'.format(hi))
        # TODO: do some trig, do this right
        core = 'translate([0,0,-outsideExtra]) cylinder(r=endTrim, h=coreThick);'
        tpl = '''@@FLIP@@intersection() {
translate([0,0,outsideExtra]) difference() {
    union() {
        @@CORE@@
        @@SHELLS@@
    }
    @@BORES@@
}
translate([-1000,-1000,0]) cube([2000,2000,coreThick]);
}
translate([0,0,outsideExtra]) union() {
@@SPHERES@@
}
'''
        data = {
            '@@SHELLS@@': '\n        '.join(shells),
            '@@BORES@@': '\n   '.join(bores),
            '@@CORE@@': core,
            '@@SPHERES@@': '\n'.join(dbspheres),
            '@@FLIP@@': '',
        }
        if flip:
            data['@@FLIP@@'] = 'translate([0,0,coreThick]) rotate(180,[1,0,0]) '
        xt = tpl
        for k,v in data.items():
            xt = xt.replace(k, v)
        out.write(xt)
        out.write('}}\nhub{}();\n'.format(hi))
        return out.getvalue()


# https://en.wikipedia.org/wiki/Regular_icosahedron#Spherical_coordinates
# alt = radians altitude of ring of points surrounding a peak at (x=0,y=0,z=1)
alt = math.atan(0.5)
cosalt = math.cos(alt)
sinalt = math.sin(alt)

def ico34():
    # top 3/4th of an icosahedron
    points = [(0,0,1)]
    edges = []
    # points[1..5] top ring
    for i in range(0,5):
        th = i * 2 * math.pi / 5
        points.append( (cosalt * math.cos(th), cosalt * math.sin(th), sinalt) )
        edges.append( (0, 1+i) )
        edges.append( (i+1, ((i+1)%5)+1) )
    # points[6..10] bottom ring
    for i in range(0,5):
        th = (i + 0.5) * 2 * math.pi / 5
        points.append( (cosalt * math.cos(th), cosalt * math.sin(th), -sinalt) )
        edges.append( (i+1, i+6) )
        edges.append( (i+1, ((i+4)%5)+6) )
        edges.append( (i+6, ((i+1)%5)+6) )
    return Wireframe(points, edges)

def ico34geo2v(rings=9):
    # construct the top 3/4th of an icosahedron, divide each edge by two
    points = [(0,0,1)]
    edges = []
    # points[1..5] top ring
    topring = []
    for i in range(0,5):
        th = i * 2 * math.pi / 5
        pt = (cosalt * math.cos(th), cosalt * math.sin(th), sinalt)
        mp = norm(mid(pt, points[0]))
        points.append(mp)
        topring.append(pt)
        edges.append( (0, 1+i) ) # peak to point
        edges.append( (i+1, ((i+1)%5)+1) ) # point to point in ring
    #logger.debug('ring 1, %d points %d edges', len(points), len(edges))
    if rings == 1:
        return Wireframe(points, edges)
    # points[6..15] next ring
    for i in range(0,5):
        pt = topring[i]
        ni = (i+1) % 5
        npt = topring[ni]
        mp = norm(mid(pt, npt))
        points.append(pt)
        points.append(mp)
        edges.append( (i+1, (i*2)+6) ) # down along ico edge
        edges.append( (i+1, (i*2)+7) ) # down to next point
        edges.append( (i+1, (((i*2)+9)%10)+6) ) # down to previous point
        edges.append( ((i*2)+6, (i*2)+7) ) # horizontal 1
        edges.append( ((i*2)+7,(((i*2)+2)%10)+6) ) # horizontal 2
    #logger.debug('ring 2, %d points %d edges', len(points), len(edges))
    if rings == 2:
        return Wireframe(points, edges)
    botring = []
    for i in range(0,5):
        th = (i + 0.5) * 2 * math.pi / 5
        pt = (cosalt * math.cos(th), cosalt * math.sin(th), -sinalt)
        botring.append(pt)
    for i in range(0,5):
        uppt = topring[i]
        pt = botring[i]
        mp = norm(mid(pt, uppt))
        points.append(mp)
        uppt2 = topring[(i+1)%5]
        mp2 = norm(mid(pt, uppt2))
        points.append(mp2)
        edges.append( ((i*2)+6, (i*2)+16) ) # down a
        edges.append( ((i*2)+7, (i*2)+17) ) # down b
        edges.append( ((i*2)+7, (i*2)+16) )
        edges.append( ((i*2)+6, (((i*2)+9)%10)+16) )
        edges.append( ((i*2)+16, (i*2)+17) )
        edges.append( ((i*2)+17, (((i*2)+2)%10)+16) )
    #logger.debug('ring 3, %d points %d edges', len(points), len(edges))
    if rings == 3:
        return Wireframe(points, edges)
    # points[26..35] next ring finishing 3/4 icosahedron
    for i in range(0,5):
        pt = botring[i]
        ni = (i+1)%5
        npt = botring[ni]
        mp = norm(mid(pt, npt))
        points.append(pt)
        points.append(mp)
    for i in range(0,10):
        ni = (i+1) % 10
        edges.append( (16+i, 26+i) )
        edges.append( (16+ni, 26+i) )
        edges.append( (26+i, 26+ni) )
    return Wireframe(points, edges)

def parseLength(x):
    if x is None:
        return None
    if x.endswith('in'):
        return float(x[:-2]) * 25.4
    if x.endswith('ft'):
        return float(x[:-2]) * 25.4 * 12
    if x.endswith('mm'):
        x = x[:-2]
    return float(x)

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--hub', type=int, default=None, help='single hub to generate scad for')
    ap.add_argument('--ico', action='store_true', default=False, help='icosahedron instead of 2v geodesic')
    ap.add_argument('--strut-dia', default=str(25.4/2), help='strut diameter, default 25.4/2mm, may specify "XXin" inches')
    ap.add_argument('--socket-length', default=None, help='socket interior length, default (strut-dia * 2)')
    ap.add_argument('--core-hole-dia', default=None, help='core hole dia, in or mm')
    ap.add_argument('--thick', default=None, help='socket wall thickness, in or mm')
    ap.add_argument('--outside-extra', default=None, help='extra height on outside of hub')
    ap.add_argument('--flip', default=False, action='store_true')
    ap.add_argument('--report', default=False, action='store_true')
    ap.add_argument('--stock-length', default=None, help='how long does strut stock come in, ft/in/mm')
    ap.add_argument('--rings', type=int, default=999, help='how many rings of a geodesic dome do you want rendered (1 = just the top pentagon)')
    ap.add_argument('-v', '--verbose', action='store_true', default=False)
    ap.add_argument('--debug-spheres', default=False, action='store_true')
    args = ap.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    if args.ico:
        ico = ico34()
    else:
        ico = ico34geo2v(rings=args.rings)
    strutDia = parseLength(args.strut_dia)
    socketLength = parseLength(args.socket_length)
    coreHoleDia = parseLength(args.core_hole_dia)
    if args.hub is not None:
        plist = [args.hub]
    else:
        plist = range(len(ico.points))
    for pi in plist:
        fname = 'h{}.scad'.format(pi)
        with open(fname, 'wt') as fout:
            fout.write(ico.hubOpenSCAD(pi, strutDia=strutDia, thick=parseLength(args.thick), socketLength=socketLength, coreHoleDia=coreHoleDia, outsideExtra=parseLength(args.outside_extra) or 0, flip=args.flip, debugSpheres=args.debug_spheres))
            print('wrote {}'.format(fname))
    if args.report:
        elengths = ico.edgeLengths()
        total = 0
        for el, count in elengths:
            total += el * count
        print('{}mm end trim'.format(ico._endTrim))
        print(elengths)
        print('{} total'.format(total))
        zs = [x[2] for x in ico.points]
        minz = min(zs)
        maxz = max(zs)
        print('height {}r'.format(maxz-minz))
        if args.stock_length:
            print(ico.stockHack(parseLength(args.stock_length)))

if __name__ == '__main__':
    main()
