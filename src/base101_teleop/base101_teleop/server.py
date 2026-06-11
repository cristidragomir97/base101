#!/usr/bin/env python3
"""Minimal web teleop for the base101 dual-arm robot.

Serves a single slider page (default http://localhost:8700/) and publishes
straight to the existing controller command topics:

    /tower_controller/commands          [lift, head_pan, head_tilt]
    /left_arm_controller/commands       [left_arm_1 .. left_arm_5]
    /right_arm_controller/commands      [right_arm_1 .. right_arm_5]
    /left_gripper_controller/commands   [left_arm_6]
    /right_gripper_controller/commands  [right_arm_6]
    /cmd_vel_key                        base Twist (via twist_mux, priority 90)

The page builds itself from /state (a joint_states snapshot), so sections for
hardware that isn't loaded (no tower, no arms) simply don't appear. Base
driving uses hold-to-move buttons republished at 10 Hz; twist_mux's 0.5 s
timeout acts as the dead-man stop.

No rosbridge, no websockets — plain HTTP polling, stdlib only.
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# target name -> (topic, ordered joint list). The UI keeps a value per joint
# and always publishes the full group array.
GROUPS = {
    'tower':         ('/tower_controller/commands', ['lift', 'head_pan', 'head_tilt']),
    'left_arm':      ('/left_arm_controller/commands',
                      ['left_arm_1', 'left_arm_2', 'left_arm_3', 'left_arm_4', 'left_arm_5']),
    'right_arm':     ('/right_arm_controller/commands',
                      ['right_arm_1', 'right_arm_2', 'right_arm_3', 'right_arm_4', 'right_arm_5']),
    'left_gripper':  ('/left_gripper_controller/commands', ['left_arm_6']),
    'right_gripper': ('/right_gripper_controller/commands', ['right_arm_6']),
}

# slider ranges per joint name (min, max); anything not listed gets +/-pi.
RANGES = {
    'lift': (-0.26, 0.26),
    'head_tilt': (-1.57, 1.57),
    'left_arm_6': (0.0, 2.14),
    'right_arm_6': (0.0, 2.14),
}


class TeleopNode(Node):
    def __init__(self):
        super().__init__('base101_teleop')
        self.declare_parameter('port', 8700)
        self.joint_states = {}
        self.pubs = {name: self.create_publisher(Float64MultiArray, topic, 1)
                     for name, (topic, _) in GROUPS.items()}
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel_key', 1)
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)

    def _on_js(self, msg):
        self.joint_states.update(zip(msg.name, msg.position))

    def command(self, body):
        target = body['target']
        if target == 'base':
            t = Twist()
            t.linear.x = float(body.get('linear', 0.0))
            t.angular.z = float(body.get('angular', 0.0))
            self.twist_pub.publish(t)
        else:
            self.pubs[target].publish(
                Float64MultiArray(data=[float(v) for v in body['data']]))


PAGE = """<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>base101 teleop</title>
<style>
  body { font-family: system-ui, sans-serif; background:#16181d; color:#dde;
         max-width: 760px; margin: 0 auto; padding: 0 12px 40px; }
  h1 { font-size: 1.1em; } h2 { font-size: .95em; color:#9ab; margin: 18px 0 6px;
       border-bottom: 1px solid #333; padding-bottom: 4px;}
  .joint { display:flex; align-items:center; gap:10px; margin:4px 0; }
  .joint label { width: 110px; font-size:.85em; color:#bcd; }
  .joint input[type=range] { flex:1; }
  .joint .val { width: 60px; text-align:right; font-variant-numeric: tabular-nums;
                font-size:.85em; color:#8fc; }
  #base { display:grid; grid-template-columns:repeat(3, 64px); gap:6px;
          justify-content:center; margin:10px 0; }
  #base button { height:48px; font-size:1.2em; background:#2a2f3a; color:#dde;
                 border:1px solid #444; border-radius:8px; touch-action:none; }
  #base button:active { background:#3d6; color:#000; }
  .row { display:flex; align-items:center; gap:10px; justify-content:center; }
  .muted { color:#778; font-size:.8em; }
  button.small { background:#2a2f3a; color:#dde; border:1px solid #444;
                 border-radius:6px; padding:4px 10px; }
</style></head><body>
<h1>base101 teleop</h1>
<div class="row"><button class="small" onclick="sync()">&#8635; sync sliders to robot</button>
<span class="muted" id="status">connecting&hellip;</span></div>
<div id="ui"></div>
<script>
const GROUPS = __GROUPS__;
const RANGES = __RANGES__;
let joints = {};            // last /state snapshot
let values = {};            // slider values keyed by joint
let speed = 0.5;

function send(body) {
  fetch('/cmd', {method:'POST', body: JSON.stringify(body)});
}
function sendGroup(g) {
  send({target:g, data: GROUPS[g][1].map(j => values[j] ?? 0)});
}
function slider(g, j) {
  const [lo, hi] = RANGES[j] || [-3.14, 3.14];
  return `<div class="joint"><label>${j}</label>
    <input type="range" min="${lo}" max="${hi}" step="0.01" value="${values[j]||0}"
      id="s_${j}" oninput="values['${j}']=+this.value;
      document.getElementById('v_${j}').textContent=(+this.value).toFixed(2);
      sendGroup('${g}')">
    <span class="val" id="v_${j}">${(values[j]||0).toFixed(2)}</span></div>`;
}
function build() {
  let h = `<h2>base</h2><div id="base">
    <span></span><button data-l="1" data-a="0">&#8593;</button><span></span>
    <button data-l="0" data-a="1">&#8634;</button>
    <button data-l="0" data-a="0" style="visibility:hidden"></button>
    <button data-l="0" data-a="-1">&#8635;</button>
    <span></span><button data-l="-1" data-a="0">&#8595;</button><span></span></div>
    <div class="joint"><label>speed</label>
    <input type="range" min="0.1" max="1.0" step="0.05" value="0.5"
      oninput="speed=+this.value;document.getElementById('v_speed').textContent=(+this.value).toFixed(2)">
    <span class="val" id="v_speed">0.50</span></div>`;
  for (const [g, [topic, js]] of Object.entries(GROUPS)) {
    if (!js.some(j => j in joints)) continue;   // hardware not present
    h += `<h2>${g.replace('_',' ')}</h2>` + js.map(j => slider(g, j)).join('');
  }
  document.getElementById('ui').innerHTML = h;
  // hold-to-drive: publish at 10 Hz while pressed, stop on release
  let timer = null;
  document.querySelectorAll('#base button').forEach(b => {
    const go = e => { e.preventDefault();
      const tw = () => send({target:'base', linear: speed*+b.dataset.l,
                             angular: 1.5*speed*+b.dataset.a});
      tw(); timer = setInterval(tw, 100); };
    const stop = () => { if (timer) clearInterval(timer); timer = null;
      send({target:'base', linear:0, angular:0}); };
    b.addEventListener('pointerdown', go);
    b.addEventListener('pointerup', stop);
    b.addEventListener('pointerleave', stop);
  });
}
async function sync() {
  joints = await (await fetch('/state')).json();
  for (const [g, [t, js]] of Object.entries(GROUPS))
    js.forEach(j => { if (j in joints) values[j] = +joints[j].toFixed(2); });
  build();
  document.getElementById('status').textContent =
    Object.keys(joints).length + ' joints';
}
sync();
</script></body></html>
"""
PAGE = (PAGE
        .replace('__GROUPS__', json.dumps(GROUPS))
        .replace('__RANGES__', json.dumps(RANGES)))


def make_handler(node):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _reply(self, code, body, ctype='application/json'):
            data = body.encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/state':
                self._reply(200, json.dumps(node.joint_states))
            else:
                self._reply(200, PAGE, 'text/html')

        def do_POST(self):
            if self.path != '/cmd':
                return self._reply(404, '{}')
            length = int(self.headers.get('Content-Length', 0))
            try:
                node.command(json.loads(self.rfile.read(length)))
                self._reply(200, '{}')
            except Exception as e:  # bad payload — report, keep serving
                self._reply(400, json.dumps({'error': str(e)}))
    return Handler


def main():
    rclpy.init()
    node = TeleopNode()
    port = node.get_parameter('port').value
    server = ThreadingHTTPServer(('0.0.0.0', port), make_handler(node))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f'base101 teleop on http://localhost:{port}/')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == '__main__':
    main()
