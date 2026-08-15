"use strict";

// JointCommandViewer
// ------------------
// One panel with a position slider for every controlled joint of the base101
// dual-arm robot: lift + pan/tilt head, both mod101 arms and both grippers.
// It is a real *subscriber* viewer on /joint_states (so it knows what
// hardware is loaded and shows live positions) that also *publishes*
// std_msgs/Float64MultiArray group commands over the rosboard WS via MSG_PUB,
// exactly like the Teleop card does for Twist.
//
// Groups whose joints aren't present in /joint_states (no tower, no arms)
// simply don't render. Sliders are initialised from the robot pose on first
// data and after pressing "sync"; while idle, the small readout next to each
// slider keeps showing the live joint position.
//
// Constructed from the "Joint sliders" entry in the System nav (it is not
// auto-selected for JointState topics — JointStateViewer stays the default).

class JointCommandViewer extends Viewer {

  onCreate() {
    // controller groups: title -> [command topic, ordered joints]
    // Groups for hardware that isn't loaded hide themselves (see onData), so
    // every variant's groups can live here side by side: the single-arm
    // base101_arm stack and the tower/dual-arm stack never coexist.
    this.groups = {
      "arm":           ["/arm_controller/commands",
                        ["arm_joint_base", "arm_joint_shoulder", "arm_joint_elbow",
                         "arm_joint_wrist_tilt", "arm_joint_wrist_roll"]],
      "gripper":       ["/gripper_controller/commands", ["arm_6"]],
      "tower":         ["/tower_controller/commands",
                        ["lift", "head_pan", "head_tilt"]],
      "left arm":      ["/left_arm_controller/commands",
                        ["left_arm_1", "left_arm_2", "left_arm_3", "left_arm_4", "left_arm_5"]],
      "left gripper":  ["/left_gripper_controller/commands", ["left_arm_6"]],
      "right arm":     ["/right_arm_controller/commands",
                        ["right_arm_1", "right_arm_2", "right_arm_3", "right_arm_4", "right_arm_5"]],
      "right gripper": ["/right_gripper_controller/commands", ["right_arm_6"]],
    };
    // slider ranges; joints not listed get +/- pi
    this.ranges = {
      "arm_6": [0.0, 2.14],
      "lift": [-0.26, 0.26],
      "head_tilt": [-1.57, 1.57],
      "left_arm_6": [0.0, 2.14],
      "right_arm_6": [0.0, 2.14],
    };

    this.card.title.text("Joint sliders");

    this.viewerNode = $('<div></div>')
      .css({"padding": "8pt 12pt 12pt", "font-size": "11px"})
      .appendTo(this.card.content);

    this.syncButton = $('<button>&#8635; sync sliders to robot</button>').css({
      "background": "#2a2f3a", "color": "#cfcfcf",
      "border": "1px solid #444", "border-radius": "6px",
      "padding": "4px 10px", "cursor": "pointer",
      "margin-bottom": "4px",
    }).appendTo(this.viewerNode);

    this.groupsNode = $('<div></div>').appendTo(this.viewerNode);

    this.sliders = {};        // joint -> {input, cmdLabel, liveLabel}
    this.values = {};         // joint -> commanded value (slider state)
    this.live = {};           // joint -> last /joint_states position
    this.built = false;

    this.syncButton.on('click', () => this._syncFromRobot());
  }

  onData(msg) {
    this.card.title.text("Joint sliders");
    if(!msg.name || !msg.position) return;
    for(let i = 0; i < msg.name.length; i++) {
      this.live[msg.name[i]] = msg.position[i];
    }
    if(!this.built) {
      this._build();
      if(this.built) this._syncFromRobot();
      return;
    }
    // keep the live readouts fresh
    for(let j in this.sliders) {
      if(j in this.live) {
        this.sliders[j].liveLabel.text(this.live[j].toFixed(2));
      }
    }
  }

  _build() {
    let any = false;
    for(let title in this.groups) {
      let [topic, joints] = this.groups[title];
      if(!joints.some(j => j in this.live)) continue;  // hardware not loaded
      any = true;

      $('<div></div>').css({
        "color": "#9ab", "font-size": "10px",
        "letter-spacing": "0.08em", "text-transform": "uppercase",
        "border-bottom": "1px solid #333",
        "margin": "10px 0 4px", "padding-bottom": "2px",
      }).text(title).appendTo(this.groupsNode);

      joints.forEach(j => this._addRow(title, j));
    }
    this.built = any;
  }

  _addRow(group, joint) {
    let [lo, hi] = this.ranges[joint] || [-3.14, 3.14];
    let row = $('<div></div>').css({
      "display": "flex", "align-items": "center", "gap": "8px",
      "margin": "2px 0",
    }).appendTo(this.groupsNode);

    $('<div></div>').css({
      "width": "90px", "color": "#bcd", "overflow": "hidden",
      "text-overflow": "ellipsis", "white-space": "nowrap",
    }).text(joint).appendTo(row);

    let input = $('<input type="range">')
      .attr({min: lo, max: hi, step: 0.01, value: 0})
      .css({"flex": "1", "min-width": "80px"})
      .appendTo(row);

    let cmdLabel = $('<div></div>').css({
      "width": "44px", "text-align": "right", "color": "#8fc",
      "font-family": "'JetBrains Mono', monospace",
    }).text("0.00").appendTo(row);

    let liveLabel = $('<div></div>').css({
      "width": "44px", "text-align": "right", "color": "#777",
      "font-family": "'JetBrains Mono', monospace",
    }).text("--").appendTo(row);

    input.on('input', () => {
      this.values[joint] = Number(input.val());
      cmdLabel.text(this.values[joint].toFixed(2));
      this._publishGroup(group);
    });

    this.sliders[joint] = {input: input, cmdLabel: cmdLabel, liveLabel: liveLabel};
    this.values[joint] = 0.0;
  }

  _syncFromRobot() {
    for(let j in this.sliders) {
      if(!(j in this.live)) continue;
      this.values[j] = this.live[j];
      this.sliders[j].input.val(this.live[j]);
      this.sliders[j].cmdLabel.text(this.live[j].toFixed(2));
    }
  }

  _publishGroup(title) {
    if(typeof currentTransport === 'undefined' || !currentTransport || !currentTransport.publish) return;
    let [topic, joints] = this.groups[title];
    currentTransport.publish({
      topicName: topic,
      topicType: "std_msgs/msg/Float64MultiArray",
      msg: {data: joints.map(j => this.values[j] ?? 0.0)},
    });
  }
}

JointCommandViewer.friendlyName = "Joint sliders (position command)";
// Not auto-selected for JointState topics — JointStateViewer remains the
// default; this card is constructed explicitly from the System nav entry.
JointCommandViewer.supportedTypes = [];
JointCommandViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(JointCommandViewer);
