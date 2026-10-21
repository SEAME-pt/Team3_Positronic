#!/usr/bin/env python3

"""
Build a Gazebo/RViz-ready myCobot 280 Jetson Nano model from Elephant Robotics' original files.

"""

import math
import os
import shutil
import struct
import sys

from lxml import etree
import numpy as np
from PIL import Image
from scipy.spatial import ConvexHull

HERE = os.path.dirname(os.path.abspath(__file__))

def _required_env(name, what):
    value = os.environ.get(name)
    if not value:
        sys.exit(f'set {name} to {what}')
    return value

sys.path.insert(
    0, _required_env('MESH_TOOLS_DIR', 'the folder with dae.py, dae2.py, fk.py, qem.py'))
from dae import mesh_props, NS  
from dae2 import apply_T, fix_winding, load_parts, write_dae  
from fk import T  
from qem import decimate, weld  

SRC = _required_env('SRC_URDF_DIR', "Elephant Robotics' original robot_model/urdf folder")
J5_SOURCE = _required_env('J5_SOURCE', 'the joint5.dae mesh')

OUT = os.environ.get('OUT_DIR', HERE)
VISUAL_FORMAT = os.environ.get(
    'VISUAL_FORMAT', 'dae'
)  
PKG = 'positronic_description/urdf/mycobot_280_jn_sim'

ARM_TOTAL_MASS = 1.030  
GRIPPER_MASS = 0.110  

CAMERA_MASS = 0.03  

SIM_GRAVITY_OFF = True

JOINT_EFFORT = 10.0  

JOINT_VELOCITY = round(math.radians(150.0), 4)  
GRIPPER_JOINT_VELOCITY = 1.0  

J5_TARGET_TRIS = 10000

J5_CLOSE_TOP_GAP = 0.0
J5_TUBE_DIAM_ORIG = 0.0386  
J5_DIAM_AT_J4_END = (
    0.0386  
)
WHITE = 0.96
GREY = 0.80
RIM = 0.25

H = 1.5708

def pose(xyz, rpy):
    return T(xyz, rpy)

ARM = [
    ('joint1', f'{SRC}/mycobot_280_jn/joint1_jet.dae', (0, 0, 0), (0, 0, 3.14159)),
    ('joint2', f'{SRC}/mycobot_280_jn/joint2.dae', (0, 0, -0.06096), (0, 0, -H)),
    ('joint3', f'{SRC}/mycobot_280_jn/joint3.dae', (0, 0, 0.03256), (0, -H, 0)),
    ('joint4', f'{SRC}/mycobot_280_jn/joint4.dae', (0, 0, 0.03056), (0, -H, 0)),
    (
        'joint5',
        J5_SOURCE,
        (0, 0, -0.03356),
        (-H, 0, 0),
    ),  
    ('joint6', f'{SRC}/mycobot_280_jn/joint6.dae', (0, 0, -0.038), (0, 0, 0)),
    ('joint6_flange', f'{SRC}/mycobot_280_jn/joint7.dae', (0, 0, -0.012), (0, 0, 0)),
]
GRIP = [
    (
        'gripper_base',
        f'{SRC}/adaptive_gripper/gripper_base.dae',
        (0, 0, -0.012),
        (0, 0, 0),
    ),
    (
        'gripper_left1',
        f'{SRC}/adaptive_gripper/gripper_left1.dae',
        (0.039, -0.0133, -0.012),
        (0, 0, 0),
    ),
    (
        'gripper_left2',
        f'{SRC}/adaptive_gripper/gripper_left2.dae',
        (0.005, -0.0195, -0.012),
        (0, 0, 0),
    ),
    (
        'gripper_left3',
        f'{SRC}/adaptive_gripper/gripper_left3.dae',
        (0.012, 0.0025, -0.012),
        (0, 0, 0),
    ),
    (
        'gripper_right1',
        f'{SRC}/adaptive_gripper/gripper_right1.dae',
        (-0.039, -0.0133, -0.012),
        (0, 0, 0),
    ),
    (
        'gripper_right2',
        f'{SRC}/adaptive_gripper/gripper_right2.dae',
        (-0.005, -0.0195, -0.012),
        (0, 0, 0),
    ),
    (
        'gripper_right3',
        f'{SRC}/adaptive_gripper/gripper_right3.dae',
        (-0.012, 0.0025, -0.012),
        (0, 0, 0),
    ),
]
GBASE = [
    ('g_base', f'{SRC}/mycobot_280_m5/G_base.dae', (0, 0, 0), (0, 0, H))
]  
CAM = [
    (
        'camera_flange',
        f'{SRC}/mycobot_280_m5/camera_flange.dae',
        (0.024, -0.008, 0.0),
        (0, 3.14159, 0),
    )
]

RINGS = [
    ('joint3', 0.0346, 0.0240),
    ('joint4', 0.0326, 0.0216),
    ('joint5', -0.0339, 0.0216),
    ('joint6', -0.0372, 0.0192),
    ('joint6_flange', -0.0113, 0.0192),
]
ENABLE_RINGS = False  
RING_H = 0.001  
RING_PROTRUDE = 0.00015  

def write_stl(path, V, F):
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    n = np.cross(b - a, c - a)
    length = np.linalg.norm(n, axis=1, keepdims=True)
    length[length == 0] = 1
    n /= length
    with open(path, 'wb') as f:
        f.write(b'myCobot 280 JN collision hull'.ljust(80, b' '))
        f.write(struct.pack('<I', len(F)))
        rec = np.zeros(
            len(F), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]
        )
        rec['n'] = n
        rec['v'][:, 0] = a
        rec['v'][:, 1] = b
        rec['v'][:, 2] = c
        f.write(rec.tobytes())

def hull(V):
    h = ConvexHull(V)
    F = h.simplices.copy()
    c = V[h.vertices].mean(0)
    for i, (a, b, cc) in enumerate(F):
        if np.dot(np.cross(V[b] - V[a], V[cc] - V[a]), V[a] - c) < 0:
            F[i] = (a, cc, b)
    used = np.unique(F)
    remap = -np.ones(len(V), int)
    remap[used] = np.arange(len(used))
    return V[used], remap[F]

def smooth_normals(P, crease_deg=45.0):
    nt = len(P) 
    a, b, c = P[0::3], P[1::3], P[2::3]
    fn = np.cross(b - a, c - a)
    area = np.linalg.norm(fn, axis=1)
    fnu = fn / np.maximum(area[:, None], 1e-18)
    _, vid = np.unique(np.round(P / 1e-6).astype(np.int64), axis=0, return_inverse=True)
    vid = vid.reshape(-1)
    face = np.repeat(np.arange(nt), 3)
    order = np.argsort(vid, kind='stable')
    bounds = np.flatnonzero(np.diff(vid[order])) + 1
    groups = np.split(order, bounds)
    N = np.zeros_like(P)
    cs = np.cos(np.radians(crease_deg))
    for g in groups:
        fs = face[g]
        fnv = fnu[fs]
        w = area[fs]
        dots = fnv @ fnv.T
        for k, corner in enumerate(g):
            m = dots[k] > cs
            N[corner] = (fnv[m] * w[m, None]).sum(0)
    N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
    return N

def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0, 1)
    return t * t * (3 - 2 * t)

L1, L2 = (
    0.0339,
    0.0360,
)  

def taper_joint5(P):
    """
    Taper the joint-5 tubes.

    Link-5 frame: tube1 along -z (joint-4 side), tube2 along -y (joint-6 side), both axes through
    the origin. Radial size varies smoothly along the link path from J5_DIAM_AT_J4_END (joint-4
    seam) down to the original diameter (joint-6 seam), so there is no step or crease at the
    mitred elbow.
    """
    S = J5_DIAM_AT_J4_END / J5_TUBE_DIAM_ORIG
    if abs(S - 1) < 1e-6 and J5_CLOSE_TOP_GAP == 0:
        return P
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    a = np.clip(-z / L1, 0, 1)
    b = np.clip(-y / L2, 0, 1)
    d = (np.abs(z) - np.abs(y)) / (np.abs(z) + np.abs(y) + 1e-9)
    w1 = smoothstep(-0.25, 0.25, d)  
    tau = L1 * (1 - a * w1) + (L1 + L2 * b) * (
        1 - w1
    )  
    sr = 1 + (S - 1) * (1 - tau / (L1 + L2))
    Q = P.copy()
    Q[:, 0] = x * sr
    Q[:, 1] = y * (1 + (sr - 1) * w1)
    Q[:, 2] = z * (1 + (sr - 1) * (1 - w1))
    ext = (
        np.clip((-z - 0.010) / (L1 - 0.010), 0, 1) * w1
    )  
    Q[:, 2] -= J5_CLOSE_TOP_GAP * ext
    return Q

def warp_normals(P, N, f, eps=1e-5):
    J = np.zeros((len(P), 3, 3))
    for k in range(3):
        e = np.zeros(3)
        e[k] = eps
        J[:, :, k] = (f(P + e) - f(P - e)) / (2 * eps)
    Nn = np.linalg.solve(J.transpose(0, 2, 1), N[..., None])[..., 0]
    return Nn / np.maximum(np.linalg.norm(Nn, axis=1, keepdims=True), 1e-12)

def link_texture(dae_path):
    """Return (png_path, mean_rgb) for a detailed DAE texture, else (None, mean_rgb)."""
    root = etree.parse(dae_path).getroot()
    fs = [
        e.text.strip()
        for e in root.findall('.//c:library_images/c:image/c:init_from', NS)
        if e.text
    ]
    for f in fs:
        p = os.path.join(os.path.dirname(dae_path), os.path.basename(f))
        if os.path.exists(p):
            im = np.asarray(Image.open(p).convert('RGB'), dtype=float)
            mean = im.reshape(-1, 3).mean(0) / 255
            detail = im.std() > 8
            return (p if detail else None), mean
    return None, None

PAD_DEPTH = 0.0028  
PIN_RADIUS = 0.0026  
EXTRA_MATS = {
    'black': ('color', (0.04, 0.04, 0.045)),
    'silver': ('color', (0.42, 0.43, 0.46)),
}

def split_part(p, mask_tri, new_mat):
    """Split one part into (rest, selected). mask_tri: bool per triangle."""

    def take(m):
        idx = np.flatnonzero(m)
        c = (idx[:, None] * 3 + np.arange(3)).reshape(-1)
        return {
            'P': p['P'][c],
            'N': p['N'][c],
            'uv': None if p['uv'] is None else p['uv'][c],
            'idx': p['idx'],
            'area': 0,
            'lo': p['lo'],
            'hi': p['hi'],
        }

    out = []
    if (~mask_tri).any():
        q = take(~mask_tri)
        q['mat'] = p.get('mat', 'white')
        out.append(q)
    if mask_tri.any():
        q = take(mask_tri)
        q['mat'] = new_mat
        out.append(q)
    return out

def clip_plane(p, axis, sign, thresh):
    """
    Split a part's triangles with the plane sign*coord[axis] = thresh.

    Returns (outside, inside) triangle sets, where inside means sign*coord[axis] > thresh.
    Normals are interpolated; UVs are dropped.
    """
    P = p['P'].reshape(-1, 3, 3)
    N = p['N'].reshape(-1, 3, 3)
    d = sign * P[:, :, axis] - thresh
    ins_P, ins_N, out_P, out_N = [], [], [], []

    def emit(poly_P, poly_N, bucket_P, bucket_N):
        for k in range(1, len(poly_P) - 1):
            bucket_P.append([poly_P[0], poly_P[k], poly_P[k + 1]])
            bucket_N.append([poly_N[0], poly_N[k], poly_N[k + 1]])

    for t in range(len(P)):
        dt = d[t]
        if (dt >= 0).all():
            ins_P.append(P[t])
            ins_N.append(N[t])
            continue
        if (dt < 0).all():
            out_P.append(P[t])
            out_N.append(N[t])
            continue
        inP, inN, ouP, ouN = [], [], [], []
        for i in range(3):
            j = (i + 1) % 3
            a, b = dt[i], dt[j]
            (inP if a >= 0 else ouP).append(P[t][i])
            (inN if a >= 0 else ouN).append(N[t][i])
            if (a >= 0) != (b >= 0):
                u = a / (a - b)
                q = P[t][i] + u * (P[t][j] - P[t][i])
                nn = N[t][i] + u * (N[t][j] - N[t][i])
                nn /= max(np.linalg.norm(nn), 1e-12)
                inP.append(q)
                inN.append(nn)
                ouP.append(q)
                ouN.append(nn)
        emit(inP, inN, ins_P, ins_N)
        emit(ouP, ouN, out_P, out_N)

    def mk(Pl, Nl):
        return (
            None
            if not Pl
            else {
                'P': np.array(Pl).reshape(-1, 3),
                'N': np.array(Nl).reshape(-1, 3),
                'uv': None,
                'idx': p['idx'],
                'area': 0,
                'lo': p['lo'],
                'hi': p['hi'],
            }
        )

    return mk(out_P, out_N), mk(ins_P, ins_N)

def tri_centroids(P):
    return (P[0::3] + P[1::3] + P[2::3]) / 3

def tri_normals(P):
    n = np.cross(P[1::3] - P[0::3], P[2::3] - P[0::3])
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-18)

def style_parts(name, parts):
    for p in parts:
        p.setdefault('mat', 'white')
    out = parts
    if name == 'camera_flange':  
        k = int(np.argmin([len(p['P']) for p in parts]))
        parts[k]['mat'] = 'black'
    if name in (
        'gripper_left1',
        'gripper_right1',
    ):  
        k = int(np.argmin([p['area'] for p in parts]))
        sign = +1 if name.startswith('gripper_left') else -1
        p = parts[k]
        edge = (sign * p['P'][:, 0]).max()
        rest, pad = clip_plane(p, 0, sign, edge - PAD_DEPTH)
        pieces = [q for q in (rest,) if q is not None]
        for q in pieces:
            q['mat'] = p.get('mat', 'white')
        if pad is not None:
            pad['mat'] = 'black'
            pieces.append(pad)
        out = parts[:k] + pieces + parts[k + 1:]
    if name.startswith('gripper_') and name != 'gripper_base':
        
        
        new = []
        for p in out:
            c = tri_centroids(p['P'])
            n = tri_normals(p['P'])
            r = np.hypot(c[:, 0], c[:, 1])
            near = (r < PIN_RADIUS) & (np.abs(n[:, 2]) > 0.9)
            mask = np.zeros(len(c), bool)
            for sgn in (+1, -1):
                sel = near & (np.sign(n[:, 2]) == sgn)
                if sel.any():
                    zex = (sgn * c[sel, 2]).max()
                    mask |= sel & (sgn * c[:, 2] > zex - 0.0004)
            new += split_part(p, mask, 'silver') if mask.any() else [p]
        out = new
    return out

M5_BASE_TEXTURE = f'{SRC}/mycobot_280_m5/joint1.png'
LABEL_CROP = (
    42,
    328,
    258,
    566,
)  
LABEL_SIZE = (0.044, 0.040)  
LABEL_CENTER = (
    -0.002,
    0.029,
)  
WALL_RECT = (
    -0.046,
    0.046,
    0.004,
    0.054,
)  
ATLAS_PX_PER_MM = 10
PANEL_RECT = (
    -0.056,
    0.056,
    0.004,
    0.046,
)  
PANEL_PLANE_X = -0.044  
PANEL_ATLAS_Y = 520  
GPIO_CROP = (778, 337, 1003, 352)  
GPIO_ROW_SIZE = (0.040, 0.0027)  
GPIO_ROW_CENTER = (-0.0224, 0.0388)  
CONNECTORS = True  

def box_part(x0, x1, y0, y1, z0, z1, mat, name):
    """Axis-aligned box with flat normals, 12 triangles, outward winding."""
    c = np.array(
        [
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y1, z0],
            [x0, y1, z0],
            [x0, y0, z1],
            [x1, y0, z1],
            [x1, y1, z1],
            [x0, y1, z1],
        ]
    )
    quads = [
        ((0, 3, 2, 1), (0, 0, -1)),
        ((4, 5, 6, 7), (0, 0, 1)),
        ((0, 1, 5, 4), (0, -1, 0)),
        ((2, 3, 7, 6), (0, 1, 0)),
        ((1, 2, 6, 5), (1, 0, 0)),
        ((3, 0, 4, 7), (-1, 0, 0)),
    ]
    P = []
    N = []
    for q, n in quads:
        for tri in ((q[0], q[1], q[2]), (q[0], q[2], q[3])):
            for k in tri:
                P.append(c[k])
                N.append(n)
    return {
        'P': np.array(P, float),
        'N': np.array(N, float),
        'uv': None,
        'idx': name,
        'area': 0,
        'lo': c.min(0),
        'hi': c.max(0),
        'mat': mat,
    }

def cylinder_part(cy, cz, r, x0, x1, mat, name, n=28):
    """Cylinder along x (centre y,z), with end caps."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    P = []
    N = []
    for i in range(n):
        j = (i + 1) % n
        y0, z0 = cy + r * np.cos(a[i]), cz + r * np.sin(a[i])
        y1, z1 = cy + r * np.cos(a[j]), cz + r * np.sin(a[j])
        n0 = (0, np.cos(a[i]), np.sin(a[i]))
        n1 = (0, np.cos(a[j]), np.sin(a[j]))
        for p, nn in (
            ((x0, y0, z0), n0),
            ((x1, y0, z0), n0),
            ((x1, y1, z1), n1),
            ((x0, y0, z0), n0),
            ((x1, y1, z1), n1),
            ((x0, y1, z1), n1),
        ):
            P.append(p)
            N.append(nn)
        for xx, sg in ((x0, -1), (x1, 1)):
            tri = (
                [(xx, cy, cz), (xx, y0, z0), (xx, y1, z1)]
                if sg > 0
                else [(xx, cy, cz), (xx, y1, z1), (xx, y0, z0)]
            )
            for p in tri:
                P.append(p)
                N.append((sg, 0, 0))
    P = np.array(P, float)
    return {
        'P': P,
        'N': np.array(N, float),
        'uv': None,
        'idx': name,
        'area': 0,
        'lo': P.min(0),
        'hi': P.max(0),
        'mat': mat,
    }

def panel_cutouts(p):
    """Find the cut-outs of the port panel (holes in its outer face) from the housing geometry."""
    from PIL import ImageDraw
    from scipy import ndimage

    P = p['P']
    tri = P.reshape(-1, 3, 3)
    n = tri_normals(P)
    c = tri_centroids(P)
    sel = (
        (n[:, 0] < -0.97)
        & (np.abs(c[:, 0] - PANEL_PLANE_X) < 0.0015)
        & (c[:, 2] > 0.005)
    )
    y0, y1, z0, z1 = PANEL_RECT
    S = ATLAS_PX_PER_MM
    W = int((y1 - y0) * 1000 * S)
    H = int((z1 - z0) * 1000 * S)
    img = Image.new('L', (W, H), 0)
    d = ImageDraw.Draw(img)
    for t in tri[sel]:
        d.polygon(
            [(float((q[1] - y0) * 1000 * S), float((z1 - q[2]) * 1000 * S)) for q in t],
            fill=255,
        )
    solid = np.asarray(img) > 0
    ys, xs = np.nonzero(solid)
    sy0, sy1, sx0, sx1 = (
        ys.min() + 8,
        ys.max() - 8,
        xs.min() + 8,
        xs.max() - 8,
    )  
    inner = np.zeros_like(solid)
    inner[sy0:sy1, sx0:sx1] = True
    lab, nlab = ndimage.label(inner & ~solid)
    out = []
    for k in range(1, nlab + 1):
        yy, xx = np.nonzero(lab == k)
        if len(yy) < 200:
            continue
        ymin = y0 + xx.min() / S / 1000
        ymax = y0 + (xx.max() + 1) / S / 1000
        zmax = z1 - yy.min() / S / 1000
        zmin = z1 - (yy.max() + 1) / S / 1000
        w, h = (ymax - ymin) * 1000, (zmax - zmin) * 1000
        fill = len(yy) / ((xx.max() - xx.min() + 1) * (yy.max() - yy.min() + 1))
        out.append(
            {
                'y0': ymin,
                'y1': ymax,
                'z0': zmin,
                'z1': zmax,
                'w': w,
                'h': h,
                'round': (fill < 0.9 and abs(w - h) < 1.0),
            }
        )
    return out

def apply_base_decals(out_parts, mats):
    """
    Texture the base housing and add the connector blocks.

    Housing part: planar UVs on the +x wall (product label) and on the -x port panel (GPIO label
    row), plain white texel elsewhere. Then add simple connector blocks behind the panel cut-outs.
    """
    low = [
        p
        for p in out_parts
        if p['P'][:, 2].min() < 0.001
        and np.ptp(p['P'][:, 0]) > 0.1
        and np.ptp(p['P'][:, 2]) > 0.05
    ]
    if not low:
        print('base decals: housing part not found')
        return
    p = low[0]
    P = p['P']
    n = tri_normals(P)
    c = tri_centroids(P)
    S = ATLAS_PX_PER_MM
    AW, AH = 2048, 1024
    atlas = Image.new('RGB', (AW, AH), (int(WHITE * 255),) * 3)
    tex = Image.open(M5_BASE_TEXTURE).convert('RGB')
    
    y0, _y1, _z0, z1 = WALL_RECT
    lab = tex.crop(LABEL_CROP).rotate(-90, expand=True)
    lw, lh = int(LABEL_SIZE[0] * 1000 * S), int(LABEL_SIZE[1] * 1000 * S)
    lab = lab.resize((lw, lh), Image.LANCZOS)
    cx = int((LABEL_CENTER[0] - y0) * 1000 * S)
    cy = int((z1 - LABEL_CENTER[1]) * 1000 * S)
    atlas.paste(lab, (cx - lw 
    
    
    _py0, py1, _pz0, pz1 = PANEL_RECT
    row = tex.crop(GPIO_CROP)
    rw, rh = int(GPIO_ROW_SIZE[0] * 1000 * S), int(GPIO_ROW_SIZE[1] * 1000 * S)
    row = row.resize((rw, rh), Image.LANCZOS)
    rx = int((py1 - GPIO_ROW_CENTER[0]) * 1000 * S)
    ry = PANEL_ATLAS_Y + int((pz1 - GPIO_ROW_CENTER[1]) * 1000 * S)
    atlas.paste(row, (rx - rw 
    atlas.save(f'{OUT}/meshes/visual/base_atlas.png')
    white_uv = np.array([(AW - 4) / AW, 1 - (AH - 4) / AH])
    uv = np.tile(white_uv, (len(P), 1))
    wall = (n[:, 0] > 0.97) & (c[:, 0] > 0.047) & (c[:, 0] < 0.051) & (c[:, 2] > 0.005)
    panel = (
        (n[:, 0] < -0.97)
        & (np.abs(c[:, 0] - PANEL_PLANE_X) < 0.0015)
        & (c[:, 2] > 0.005)
    )
    cw = np.repeat(wall, 3)
    Pw = P[cw]
    uv[cw, 0] = (Pw[:, 1] - y0) * 1000 * S / AW
    uv[cw, 1] = 1 - (z1 - Pw[:, 2]) * 1000 * S / AH
    cp = np.repeat(panel, 3)
    Pp = P[cp]
    uv[cp, 0] = (py1 - Pp[:, 1]) * 1000 * S / AW
    uv[cp, 1] = 1 - (PANEL_ATLAS_Y + (pz1 - Pp[:, 2]) * 1000 * S) / AH
    p['uv'] = uv
    p['mat'] = 'base_tex'
    mats['base_tex'] = ('texture', 'base_atlas.png')
    print(
        f'base decals: label on {int(wall.sum())} wall tris, '
        f'GPIO row on {int(panel.sum())} panel tris'
    )
    mats['steel'] = ('color', (0.76, 0.77, 0.79))
    mats['gold'] = ('color', (0.85, 0.68, 0.25))
    if not CONNECTORS:
        return
    cuts = panel_cutouts(p)
    xf, xb = -0.0432, -0.0300
    added = 0
    for k, ct in enumerate(sorted(cuts, key=lambda t: (t['y0']))):
        if ct['w'] > 40:
            continue  
        m = 0.0004
        if ct['round'] and ct['w'] > 12:  
            cyc, czc, r = (
                (ct['y0'] + ct['y1']) / 2,
                (ct['z0'] + ct['z1']) / 2,
                ct['w'] / 2000,
            )
            out_parts.append(
                cylinder_part(cyc, czc, r - m, xf, xb, 'black', f'jack_{k}')
            )
            out_parts.append(
                cylinder_part(
                    cyc, czc, 0.0026, xf - 0.0004, xb, 'steel', f'jack_pin_{k}'
                )
            )
            added += 1
        elif ct['round']:
            continue  
        else:
            out_parts.append(
                box_part(
                    xf,
                    xb,
                    ct['y0'] + m,
                    ct['y1'] - m,
                    ct['z0'] + m,
                    ct['z1'] - m,
                    'steel',
                    f'conn_{k}',
                )
            )
            mg = min(0.0016, 0.3 * min(ct['w'], ct['h']) / 1000)
            out_parts.append(
                box_part(
                    xf - 0.0003,
                    xf + 0.0003,
                    ct['y0'] + m + mg,
                    ct['y1'] - m - mg,
                    ct['z0'] + m + mg,
                    ct['z1'] - m - mg,
                    'black',
                    f'conn_open_{k}',
                )
            )
            added += 1
    print(f'base connectors: {added} blocks behind {len(cuts)} detected cut-outs')

for sub in ('visual', 'collision'):
    shutil.rmtree(f'{OUT}/meshes/{sub}', ignore_errors=True)
    os.makedirs(f'{OUT}/meshes/{sub}')
links = {}
report = []
for name, path, xyz, rpy in ARM + GRIP + CAM + GBASE:
    M = pose(xyz, rpy)
    R = M[:3, :3]
    parts = load_parts(path)
    png, mean = link_texture(path)
    if name == 'joint5':
        Vw, Fw = weld(parts[0]['pos'])
        V2, F2 = decimate(Vw, Fw, J5_TARGET_TRIS)
        parts = [{'pos': V2[F2].reshape(-1, 3), 'nrm': None, 'uv': None, 'det': 1.0}]
        print(f'joint5 simplified: {len(Fw)} -> {len(F2)} triangles')
    out_parts = []
    allpos = []
    for k, p in enumerate(parts):
        P, N = p['pos'], p['nrm']
        if N is not None:
            P, N, nfix, idx = fix_winding(P, N)
            if nfix and p['uv'] is not None:
                p['uv'] = p['uv'][idx]
        out_parts.append({'P': P, 'N': N, 'uv': p['uv'], 'idx': k})
    
    for p in out_parts:
        a, b, c = p['P'][0::3], p['P'][1::3], p['P'][2::3]
        p['area'] = np.linalg.norm(np.cross(b - a, c - a), axis=1).sum() / 2
        p['lo'], p['hi'] = p['P'].min(0), p['P'].max(0)
    
    for p in out_parts:
        p['P'] = apply_T(p['P'], M)
    for p in out_parts:
        p['N'] = smooth_normals(p['P']) if p['N'] is None else p['N'] @ R.T
    if name == 'joint5':
        for p in out_parts:
            p['N'] = warp_normals(p['P'], p['N'], taper_joint5)
            p['P'] = taper_joint5(p['P'])
    allpos = [p['P'] for p in out_parts]
    
    mats = {
        'white': ('color', (WHITE,) * 3),
        'grey': ('color', (GREY,) * 3),
        'dark': ('color', (0.18,) * 3),
        'rim': ('color', (RIM + 0.15,) * 3),
        'pcb': ('color', (0.10, 0.42, 0.22)),
        'metal': ('color', (0.62, 0.62, 0.64)),
    }
    if png:
        shutil.copy(png, f'{OUT}/meshes/visual/{os.path.basename(png)}')
        mats['tex'] = ('texture', os.path.basename(png))
    elif mean is not None and name not in ('joint1',):
        mats['flat'] = ('color', tuple(np.clip(mean, 0, 1)))
    default = 'tex' if png else 'white'
    if name == 'joint1':
        ranked = sorted(range(len(out_parts)), key=lambda i: -out_parts[i]['area'])
        for rank, i in enumerate(ranked):
            p = out_parts[i]
            size = p['hi'] - p['lo']
            if rank in (0, 1):
                m = 'white'
            elif rank == 2:
                m = 'rim'  
            elif size[2] < 0.003 and size[0] > 0.07:
                m = 'pcb'
            elif p['area'] > 0.005:
                m = 'metal'
            else:
                m = 'dark'
            p['mat'] = m
    else:
        for p in out_parts:
            p['mat'] = default
    mats.update(EXTRA_MATS)
    if name == 'joint1':
        apply_base_decals(out_parts, mats)
    if name.startswith('gripper_') or name == 'camera_flange':
        out_parts = style_parts(name, out_parts)
    if name == 'camera_flange':
        lens = next(p for p in out_parts if p['mat'] == 'black')['P']
        CAM_THICK = float(np.ptp(np.vstack([p['P'] for p in out_parts])[:, 1]))
        LENS = (
            float(lens[:, 0].mean()),
            float(lens[:, 1].max()),
            float(lens[:, 2].mean()),
        )
        print('camera lens (link frame, m):', np.round(LENS, 4))
    usedm = {m: mats[m] for m in {p['mat'] for p in out_parts}}
    if VISUAL_FORMAT == 'dae':
        write_dae(
            f'{OUT}/meshes/visual/{name}.dae',
            [
                {
                    'name': f'{name}_{p["idx"]}',
                    'pos': p['P'],
                    'nrm': p['N'],
                    'uv': p['uv'],
                    'mat': p['mat'],
                }
                for p in out_parts
            ],
            usedm,
        )
    else:
        Vv = np.vstack([p['P'] for p in out_parts])
        write_stl(
            f'{OUT}/meshes/visual/{name}.stl', Vv, np.arange(len(Vv)).reshape(-1, 3)
        )
    V = np.vstack(allpos)
    hv, hf = hull(V)
    write_stl(f'{OUT}/meshes/collision/{name}.stl', hv, hf)
    if name == 'g_base':
        G_TOP = float(V[:, 2].max())
        print(
            f'g_base: footprint {np.ptp(V[:, 0])*1000:.0f} x {np.ptp(V[:, 1])*1000:.0f} mm, '
            f'height {G_TOP*1000:.1f} mm'
        )
    Pp = mesh_props(hv, hf)
    links[name] = {'vol': Pp['volume'], 'com': Pp['com'], 'I': Pp['I_unit_density']}
    report.append(
        f'{name:15s} parts={len(out_parts):3d} '
        f"tris={sum(len(p['P'])//3 for p in out_parts):6d} tex={'yes' if png else 'no '} "
        f'size(mm)={np.round(np.ptp(V, axis=0)*1000, 1)}'
    )
print('\n'.join(report))

def assign(group, total):
    vs = sum(links[n]['vol'] for n, *_ in group)
    for n, *_ in group:
        rho = total / vs
        links[n]['mass'] = rho * links[n]['vol']
        links[n]['Icom'] = links[n]['I'] * rho

assign(ARM + GBASE, ARM_TOTAL_MASS)
assign(GRIP, GRIPPER_MASS)
assign(CAM, CAMERA_MASS)
for n, d in links.items():
    w = np.linalg.eigvalsh(d['Icom'])
    assert (w > 0).all() and w[0] + w[1] >= w[2] * 0.999, n

def link_xml(n):
    d = links[n]
    inertia = d['Icom']
    c = d['com']
    stl_mat = (
        f'\n      <material name="m_{n}"><color rgba="{WHITE} {WHITE} {WHITE} 1"/></material>'
        if VISUAL_FORMAT == 'stl'
        else ''
    )
    rings = ''
    for ln, z, r in RINGS:
        if ENABLE_RINGS and ln == n:
            rings += (
                f'    <visual>\n      <origin xyz="0 0 {z}" rpy="0 0 0"/>\n'
                f'      <geometry><cylinder radius="{r+RING_PROTRUDE:.5f}" '
                f'length="{RING_H}"/></geometry>\n'
                f'      <material name="rim_{ln}"><color rgba="{RIM} {RIM} {RIM} 1"/></material>\n'
                f'    </visual>\n'
            )
    visual_mesh = f'package://{PKG}/meshes/visual/{n}.{VISUAL_FORMAT}'
    i = inertia
    inertia_xml = (
        f'ixx="{i[0, 0]:.4e}" ixy="{i[0, 1]:.4e}" ixz="{i[0, 2]:.4e}" '
        f'iyy="{i[1, 1]:.4e}" iyz="{i[1, 2]:.4e}" izz="{i[2, 2]:.4e}"'
    )
    return f"""  <link name="{n}">
    <visual>
      <geometry><mesh filename="{visual_mesh}"/></geometry>{stl_mat}
    </visual>
{rings}    <collision>
      <geometry><mesh filename="package://{PKG}/meshes/collision/{n}.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}" rpy="0 0 0"/>
      <mass value="{d['mass']:.5f}"/>
      <inertia {inertia_xml}/>
    </inertial>
  </link>
"""

def joint_xml(name, parent, child, xyz, rpy, lo, up, mimic=None, jtype='revolute'):
    s = (
        f'  <joint name="{name}" type="{jtype}">\n'
        f'    <parent link="{parent}"/><child link="{child}"/>\n'
        f'    <origin xyz="{xyz}" rpy="{rpy}"/>\n'
    )
    if jtype == 'revolute':
        velocity = GRIPPER_JOINT_VELOCITY if name.startswith('gripper') else JOINT_VELOCITY
        s += (
            '    <axis xyz="0 0 1"/>\n'
            f'    <limit effort="{JOINT_EFFORT}" lower="{lo}" upper="{up}" '
            f'velocity="{velocity}"/>\n'
            '    <dynamics damping="0.1" friction="0.01"/>\n'
        )
        if mimic:
            s += f'    <mimic joint="{mimic[0]}" multiplier="{mimic[1]}" offset="0"/>\n'
    return s + '  </joint>\n'

ARM_JOINTS = [
    ('joint2_to_joint1', 'joint1', 'joint2', '0 0 0.15756', '0 0 0', -2.9321, 2.9321),
    (
        'joint3_to_joint2',
        'joint2',
        'joint3',
        '0 0 -0.001',
        '0 1.5708 -1.5708',
        -2.4434,
        2.4434,
    ),
    ('joint4_to_joint3', 'joint3', 'joint4', '-0.1104 0 0', '0 0 0', -2.6179, 2.6179),
    (
        'joint5_to_joint4',
        'joint4',
        'joint5',
        '-0.096 0 0.06462',
        '0 0 -1.5708',
        -2.6179,
        2.6179,
    ),
    (
        'joint6_to_joint5',
        'joint5',
        'joint6',
        '0 -0.07318 0',
        '1.5708 -1.5708 0',
        -2.7052,
        2.7925,
    ),
    (
        'joint6output_to_joint6',
        'joint6',
        'joint6_flange',
        '0 0.0456 0',
        '-1.5708 0 0',
        -3.14159,
        3.14159,
    ),
]
GRIP_JOINTS = [
    (
        'gripper_controller',
        'gripper_base',
        'gripper_left3',
        '-0.012 0.005 0',
        '0 0 0',
        
        
        -0.74,
        0.15,
        None,
    ),
    (
        'gripper_base_to_gripper_left2',
        'gripper_base',
        'gripper_left2',
        '-0.005 0.027 0',
        '0 0 0',
        -0.8,
        0.5,
        ('gripper_controller', 1.0),
    ),
    (
        'gripper_left3_to_gripper_left1',
        'gripper_left3',
        'gripper_left1',
        '-0.027 0.016 0',
        '0 0 0',
        -0.5,
        0.8,
        ('gripper_controller', -1.0),
    ),
    (
        'gripper_base_to_gripper_right3',
        'gripper_base',
        'gripper_right3',
        '0.012 0.005 0',
        '0 0 0',
        -0.15,
        0.8,
        ('gripper_controller', -1.0),
    ),
    (
        'gripper_base_to_gripper_right2',
        'gripper_base',
        'gripper_right2',
        '0.005 0.027 0',
        '0 0 0',
        -0.5,
        0.8,
        ('gripper_controller', -1.0),
    ),
    (
        'gripper_right3_to_gripper_right1',
        'gripper_right3',
        'gripper_right1',
        '0.027 0.016 0',
        '0 0 0',
        -0.8,
        0.5,
        ('gripper_controller', 1.0),
    ),
]
CAMERA_SENSOR = f"""
  
  <link name="camera_link"/>
  <joint name="camera_flange_to_camera_link" type="fixed">
    <parent link="camera_flange"/><child link="camera_link"/>
    <origin xyz="{LENS[0]:.4f} {LENS[1]+0.0005:.4f} {LENS[2]:.4f}" rpy="0 0 1.5708"/>
  </joint>
  <gazebo reference="camera_link">
    <sensor name="wrist_camera" type="camera">
      <always_on>1</always_on><update_rate>30</update_rate><visualize>true</visualize>
      <topic>wrist_camera/image_raw</topic>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.02</near><far>5.0</far></clip>
      </camera>
    </sensor>
  </gazebo>
"""

def build(gripper, camera, g_base=True):
    s = '<?xml version="1.0"?>\n<robot name="mycobot_280_jn">\n'
    s += '  <link name="world"/>\n'
    if g_base:  
        s += (
            '  <joint name="world_to_g_base" type="fixed"><parent link="world"/>'
            '<child link="g_base"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>\n\n'
        )
        s += link_xml('g_base')
        s += (
            '  <joint name="g_base_to_joint1" type="fixed"><parent link="g_base"/>'
            f'<child link="joint1"/><origin xyz="0 0 {G_TOP:.4f}" rpy="0 0 0"/></joint>\n\n'
        )
    else:  
        s += (
            '  <joint name="world_to_joint1" type="fixed"><parent link="world"/>'
            '<child link="joint1"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>\n\n'
        )
    for n, *_ in ARM:
        s += link_xml(n)
    for j in ARM_JOINTS:
        s += joint_xml(*j)
    if gripper:
        gz = 0.034 + (
            (CAM_THICK + 0.0005) if camera else 0.0
        )  
        s += joint_xml(
            'joint6output_to_gripper_base',
            'joint6_flange',
            'gripper_base',
            f'0 0 {gz:.4f}',
            '1.5708 0 0',
            0,
            0,
            jtype='fixed',
        )
        for n, *_ in GRIP:
            s += link_xml(n)
        for j in GRIP_JOINTS:
            s += joint_xml(j[0], j[1], j[2], j[3], j[4], j[5], j[6], mimic=j[7])
    if camera:
        s += joint_xml(
            'joint6output_to_camera_flange',
            'joint6_flange',
            'camera_flange',
            '0 0 0.01',
            '1.5708 0 0',
            0,
            0,
            jtype='fixed',
        )
        s += link_xml('camera_flange') + CAMERA_SENSOR
    if SIM_GRAVITY_OFF:
        moving = (
            [n for n, *_ in ARM[1:]]
            + ([n for n, *_ in GRIP] if gripper else [])
            + (['camera_flange'] if camera else [])
        )
        s += (
            '\n  \n'
        )
        for n in moving:
            s += f'  <gazebo reference="{n}"><turnGravityOff>true</turnGravityOff></gazebo>\n'
    return s + '</robot>\n'

VARIANTS = {
    'mycobot_280_jn': (False, False),
    'mycobot_280_jn_gripper': (True, False),
    'mycobot_280_jn_camera': (False, True),
    'mycobot_280_jn_gripper_camera': (True, True),
}
for stem, (g, c) in VARIANTS.items():
    for suffix, with_base in (('', True), ('_no_base', False)):
        filename = f'{OUT}/{stem}{suffix}.urdf'

        with open(filename, 'w') as file:
            file.write(build(g, c, with_base))

        print('wrote', f'{stem}{suffix}.urdf')
