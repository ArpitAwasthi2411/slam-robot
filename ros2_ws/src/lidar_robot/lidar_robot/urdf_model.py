"""Robot model: render urdf/pathik.urdf.xacro and read its frames (pure Python, tested).

    xml = render(xacro_path, {'laser_x': '-0.13', ...})      # real xacro if installed, else the small renderer
    frames(xml) -> {'laser': {'parent': 'base_link', 'xyz': (-0.13, 0, 0.1), 'rpy': (0, 0, 0)}, ...}
    launch_args_from_params(robot_params.yaml) -> footprint / wheel / camera args for the model

The fallback renderer understands exactly what the file uses: xacro:arg (with defaults), xacro:property and
${...} expressions (math: pi, sin, cos, radians). Enough for the robot to start without the xacro package and
for the tests to check the model against the parameters.
"""
import math
import re
import xml.etree.ElementTree as ET

XACRO_NS = 'http://www.ros.org/wiki/xacro'
_ARG = re.compile(r'\$\(arg ([A-Za-z_][A-Za-z0-9_]*)\)')
_EXPR = re.compile(r'\$\{([^}]*)\}')
_SAFE = {'pi': math.pi, 'sin': math.sin, 'cos': math.cos, 'radians': math.radians, 'abs': abs}


def _fmt(v):
    return '%.9g' % v if isinstance(v, float) else str(v)


def _eval(expr, names):
    return eval(expr, {'__builtins__': {}}, dict(_SAFE, **names))   # noqa: S307 (our own file, numbers only)


def _subst(text, names):
    def one(m):
        v = _eval(m.group(1), names)
        return _fmt(float(v)) if isinstance(v, (int, float)) else str(v)
    return _EXPR.sub(one, text)


def render_simple(text, args=None):
    """Minimal xacro: args -> properties -> ${} in attributes. Returns URDF XML text."""
    args = dict(args or {})
    defaults = dict(re.findall(r'<xacro:arg\s+name="([^"]+)"\s+default="([^"]*)"\s*/>', text))
    values = {**defaults, **{k: str(v) for k, v in args.items()}}
    unknown = set(_ARG.findall(text)) - set(values)
    if unknown:
        raise ValueError(f'xacro args without a value: {sorted(unknown)}')
    text = _ARG.sub(lambda m: values[m.group(1)], text)
    ET.register_namespace('xacro', XACRO_NS)
    root = ET.fromstring(text)
    props = {}
    for el in list(root):
        if el.tag == f'{{{XACRO_NS}}}property':
            v = _subst(el.get('value', ''), props)
            try:
                props[el.get('name')] = float(v)
            except ValueError:
                props[el.get('name')] = v
            root.remove(el)
        elif el.tag == f'{{{XACRO_NS}}}arg':
            root.remove(el)
    for el in root.iter():
        for k, v in list(el.attrib.items()):
            if '${' in v:
                el.set(k, _subst(v, props))
    if any(el.tag.startswith(f'{{{XACRO_NS}}}') for el in root.iter()):
        raise ValueError('the model uses xacro features this renderer does not know: install ros-humble-xacro')
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding='unicode')


def render(path, args=None):
    """Real xacro when installed (it is on the robot: ros-humble-xacro), else render_simple."""
    try:
        import xacro
        doc = xacro.process_file(path, mappings={k: str(v) for k, v in (args or {}).items()})
        return doc.toxml()
    except ImportError:
        with open(path) as f:
            return render_simple(f.read(), args)


def frames(urdf_xml):
    """child link -> {'parent', 'xyz', 'rpy', 'type'} for every joint."""
    root = ET.fromstring(urdf_xml.split('?>', 1)[-1] if urdf_xml.lstrip().startswith('<?xml') else urdf_xml)
    out = {}
    for j in root.findall('joint'):
        o = j.find('origin')
        xyz = tuple(float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split())
        rpy = tuple(float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split())
        out[j.find('child').get('link')] = {'parent': j.find('parent').get('link'), 'xyz': xyz, 'rpy': rpy,
                                            'type': j.get('type')}
    return out


def links(urdf_xml):
    root = ET.fromstring(urdf_xml.split('?>', 1)[-1] if urdf_xml.lstrip().startswith('<?xml') else urdf_xml)
    return [ln.get('name') for ln in root.findall('link')]


def launch_args_from_params(params):
    """config/robot_params.yaml (dict) -> the model args that live there (body, wheels, camera)."""
    nav = (params.get('navigator') or {}).get('ros__parameters', {})
    bridge = (params.get('esp32_bridge') or {}).get('ros__parameters', {})
    out = {}
    for k in ('footprint_front', 'footprint_rear', 'footprint_half_width'):
        if k in nav:
            out[k] = nav[k]
    for k in ('camera_x', 'camera_y', 'camera_z'):
        if k in nav:
            out[k] = nav[k]
    for k in ('wheel_radius', 'wheel_separation'):
        if k in bridge:
            out[k] = bridge[k]
    return {k: str(v) for k, v in out.items()}
