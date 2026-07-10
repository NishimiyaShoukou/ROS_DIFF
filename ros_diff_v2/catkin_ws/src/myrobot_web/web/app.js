(function () {
  "use strict";

  const $ = (selector) => document.querySelector(selector);
  const $$ = (selector) => Array.from(document.querySelectorAll(selector));

  const elements = {
    rosStatus: $("#ros-status"),
    modeStatus: $("#mode-status"),
    emergencyStop: $("#emergency-stop"),
    modeButtons: $$('[data-start-mode]'),
    saveMap: $("#save-map"),
    stopMode: $("#stop-mode"),
    canvas: $("#map-canvas"),
    viewport: $("#map-viewport"),
    mapEmpty: $("#map-empty"),
    mapSize: $("#map-size"),
    poseReadout: $("#pose-readout"),
    mapToolButtons: $$('[data-map-tool]'),
    zoomOut: $("#zoom-out"),
    zoomIn: $("#zoom-in"),
    fitMap: $("#fit-map"),
    joystick: $("#joystick"),
    joystickKnob: $("#joystick-knob"),
    driveState: $("#drive-state"),
    speedButtons: $$('[data-speed]'),
    linearValue: $("#linear-value"),
    angularValue: $("#angular-value"),
    batterySummary: $("#battery-summary"),
    systemMessage: $("#system-message"),
    saveDialog: $("#save-dialog"),
    confirmSave: $("#confirm-save"),
    toast: $("#toast"),
  };

  const nodeElements = {
    base: $("#node-base"),
    lidar: $("#node-lidar"),
    mapping: $("#node-mapping"),
    localization: $("#node-localization"),
    navigation: $("#node-navigation"),
  };

  const modeLabels = {
    stopped: "已停止",
    idle: "底盘",
    mapping: "建图",
    navigation: "导航",
    external_idle: "外部底盘",
    external_mapping: "外部建图",
    external_navigation: "外部导航",
  };

  const speedPresets = {
    precise: { linear: 0.06, angular: 0.22 },
    normal: { linear: 0.10, angular: 0.40 },
  };

  const state = {
    ros: null,
    connected: false,
    reconnectTimer: null,
    interfaces: null,
    manager: {
      mode: "stopped",
      managed: false,
      map_available: false,
      nodes: {},
      error: "",
    },
    busy: false,
    map: null,
    pose: null,
    path: [],
    fitted: false,
    view: { scale: 1, offsetX: 0, offsetY: 0 },
    mapTool: "view",
    draft: null,
    actionPointer: null,
    viewPointers: new Map(),
    panLast: null,
    pinch: null,
    speedPreset: "precise",
    joystick: {
      active: false,
      pointerId: null,
      linear: 0,
      angular: 0,
    },
    toastTimer: null,
  };

  function websocketUrl() {
    const params = new URLSearchParams(window.location.search);
    const port = params.get("ws") || "9090";
    const scheme = window.location.protocol === "https:" ? "wss" : "ws";
    return `${scheme}://${window.location.hostname}:${port}`;
  }

  function connectRos() {
    if (typeof window.ROSLIB === "undefined") {
      setSystemMessage("roslibjs 未加载，检查 web/vendor/roslib.min.js", true);
      return;
    }

    clearTimeout(state.reconnectTimer);
    const ros = new ROSLIB.Ros({ url: websocketUrl() });
    state.ros = ros;

    ros.on("connection", function () {
      state.connected = true;
      elements.rosStatus.classList.remove("status-offline");
      elements.rosStatus.classList.add("status-online");
      elements.rosStatus.lastChild.textContent = "ROS 已连接";
      initializeRosInterfaces(ros);
      setSystemMessage("控制链路已连接", false);
      updateControls();
    });

    ros.on("error", function () {
      setSystemMessage("无法连接树莓派 ROS WebSocket", true);
    });

    ros.on("close", function () {
      if (state.ros !== ros) return;
      state.connected = false;
      state.interfaces = null;
      elements.rosStatus.classList.remove("status-online");
      elements.rosStatus.classList.add("status-offline");
      elements.rosStatus.lastChild.textContent = "ROS 未连接";
      releaseJoystick();
      updateControls();
      state.reconnectTimer = window.setTimeout(connectRos, 2000);
    });
  }

  function initializeRosInterfaces(ros) {
    const interfaces = {
      manualCmd: new ROSLIB.Topic({
        ros,
        name: "/mobile/cmd_vel",
        messageType: "geometry_msgs/Twist",
      }),
      initialPose: new ROSLIB.Topic({
        ros,
        name: "/initialpose",
        messageType: "geometry_msgs/PoseWithCovarianceStamped",
      }),
      navigationGoal: new ROSLIB.Topic({
        ros,
        name: "/move_base_simple/goal",
        messageType: "geometry_msgs/PoseStamped",
      }),
      navigationCancel: new ROSLIB.Topic({
        ros,
        name: "/move_base/cancel",
        messageType: "actionlib_msgs/GoalID",
      }),
      services: {},
    };

    ["start_idle", "start_mapping", "start_navigation", "stop_mode", "save_map", "emergency_stop"]
      .forEach(function (name) {
        interfaces.services[name] = new ROSLIB.Service({
          ros,
          name: `/mobile/${name}`,
          serviceType: "std_srvs/Trigger",
        });
      });

    new ROSLIB.Topic({
      ros,
      name: "/mobile/status",
      messageType: "std_msgs/String",
      throttle_rate: 250,
      queue_length: 1,
    }).subscribe(handleManagerStatus);

    new ROSLIB.Topic({
      ros,
      name: "/mobile/robot_pose",
      messageType: "geometry_msgs/PoseStamped",
      throttle_rate: 100,
      queue_length: 1,
    }).subscribe(handleRobotPose);

    new ROSLIB.Topic({
      ros,
      name: "/map",
      messageType: "nav_msgs/OccupancyGrid",
      compression: "cbor",
      throttle_rate: 1000,
      queue_length: 1,
    }).subscribe(handleMap);

    new ROSLIB.Topic({
      ros,
      name: "/battery_state",
      messageType: "sensor_msgs/BatteryState",
      throttle_rate: 1000,
      queue_length: 1,
    }).subscribe(handleBattery);

    new ROSLIB.Topic({
      ros,
      name: "/move_base/GlobalPlanner/plan",
      messageType: "nav_msgs/Path",
      throttle_rate: 500,
      queue_length: 1,
    }).subscribe(function (message) {
      state.path = message.poses || [];
      renderMap();
    });

    state.interfaces = interfaces;
  }

  function handleManagerStatus(message) {
    try {
      state.manager = JSON.parse(message.data);
    } catch (_error) {
      setSystemMessage("机器人状态数据格式错误", true);
      return;
    }

    const mode = state.manager.mode || "stopped";
    elements.modeStatus.lastChild.textContent = modeLabels[mode] || mode;
    elements.modeStatus.classList.toggle("status-online", mode !== "stopped");

    const baseMode = mode.replace("external_", "");
    if (baseMode !== "navigation" && state.path.length) {
      state.path = [];
      renderMap();
    }
    elements.modeButtons.forEach(function (button) {
      button.classList.toggle("active", button.dataset.startMode === baseMode);
    });

    Object.keys(nodeElements).forEach(function (name) {
      const active = Boolean(state.manager.nodes && state.manager.nodes[name]);
      nodeElements[name].textContent = active ? "运行中" : "未运行";
      nodeElements[name].dataset.state = active ? "on" : "off";
    });

    if (state.manager.error) {
      setSystemMessage(state.manager.error, true);
    } else if (mode.startsWith("external_")) {
      setSystemMessage("当前节点由外部终端启动，网页不会重复接管", false);
    } else {
      setSystemMessage(`当前模式：${modeLabels[mode] || mode}`, false);
    }
    updateControls();
  }

  function handleRobotPose(message) {
    state.pose = message;
    const position = message.pose.position;
    const yaw = quaternionYaw(message.pose.orientation);
    elements.poseReadout.textContent = `x ${position.x.toFixed(2)} · y ${position.y.toFixed(2)} · θ ${(yaw * 180 / Math.PI).toFixed(0)}°`;
    renderMap();
  }

  function handleBattery(message) {
    const voltage = Number(message.voltage);
    const percentage = Number(message.percentage);
    const parts = [];
    if (Number.isFinite(percentage) && percentage >= 0) {
      parts.push(`${Math.round(percentage * 100)}%`);
    }
    if (Number.isFinite(voltage) && voltage > 0) {
      parts.push(`${voltage.toFixed(1)} V`);
    }
    elements.batterySummary.textContent = parts.length ? `电池 ${parts.join(" · ")}` : "电池 --";
  }

  function handleMap(message) {
    const width = message.info.width;
    const height = message.info.height;
    if (!width || !height || !message.data) return;

    const imageCanvas = document.createElement("canvas");
    imageCanvas.width = width;
    imageCanvas.height = height;
    const imageContext = imageCanvas.getContext("2d", { alpha: false });
    const image = imageContext.createImageData(width, height);

    for (let y = 0; y < height; y += 1) {
      const outputY = height - y - 1;
      for (let x = 0; x < width; x += 1) {
        const occupancy = Number(message.data[y * width + x]);
        const pixel = (outputY * width + x) * 4;
        let red;
        let green;
        let blue;
        if (occupancy < 0) {
          red = 77; green = 85; blue = 83;
        } else if (occupancy >= 65) {
          red = 31; green = 36; blue = 35;
        } else {
          const shade = Math.max(205, Math.round(245 - occupancy * 0.42));
          red = shade; green = shade + 1; blue = shade;
        }
        image.data[pixel] = red;
        image.data[pixel + 1] = green;
        image.data[pixel + 2] = blue;
        image.data[pixel + 3] = 255;
      }
    }
    imageContext.putImageData(image, 0, 0);

    const changedSize = !state.map || state.map.width !== width || state.map.height !== height;
    state.map = {
      width,
      height,
      resolution: message.info.resolution,
      origin: message.info.origin,
      frameId: message.header.frame_id || "map",
      image: imageCanvas,
    };
    elements.mapEmpty.hidden = true;
    elements.mapSize.textContent = `${(width * message.info.resolution).toFixed(1)} × ${(height * message.info.resolution).toFixed(1)} m`;

    if (!state.fitted || changedSize) fitMap();
    else renderMap();
  }

  function resizeCanvas() {
    const rect = elements.viewport.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.max(1, Math.round(rect.width * dpr));
    const height = Math.max(1, Math.round(rect.height * dpr));
    if (elements.canvas.width !== width || elements.canvas.height !== height) {
      elements.canvas.width = width;
      elements.canvas.height = height;
      if (state.map && !state.fitted) fitMap();
      else renderMap();
    }
  }

  function fitMap() {
    if (!state.map) return;
    const rect = elements.viewport.getBoundingClientRect();
    const scale = Math.min(rect.width / state.map.width, rect.height / state.map.height) * 0.92;
    state.view.scale = clamp(scale, 0.05, 80);
    state.view.offsetX = (rect.width - state.map.width * state.view.scale) / 2;
    state.view.offsetY = (rect.height - state.map.height * state.view.scale) / 2;
    state.fitted = true;
    renderMap();
  }

  function renderMap() {
    const rect = elements.viewport.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const context = elements.canvas.getContext("2d");
    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    context.clearRect(0, 0, rect.width, rect.height);
    context.fillStyle = "#303736";
    context.fillRect(0, 0, rect.width, rect.height);
    if (!state.map) return;

    context.save();
    context.translate(state.view.offsetX, state.view.offsetY);
    context.scale(state.view.scale, state.view.scale);
    context.imageSmoothingEnabled = false;
    context.drawImage(state.map.image, 0, 0);
    drawPath(context);
    context.restore();

    if (state.draft) drawDirectionArrow(context, state.draft.start, state.draft.end, "#d58a1d", false);
    if (state.pose && state.pose.header.frame_id === state.map.frameId) drawRobot(context);
  }

  function drawPath(context) {
    if (!state.path.length || !state.map) return;
    context.beginPath();
    state.path.forEach(function (pose, index) {
      const point = worldToMapPixel(pose.pose.position.x, pose.pose.position.y);
      if (index === 0) context.moveTo(point.x, point.y);
      else context.lineTo(point.x, point.y);
    });
    context.lineWidth = 2 / state.view.scale;
    context.strokeStyle = "#0b8f82";
    context.stroke();
  }

  function drawRobot(context) {
    const position = state.pose.pose.position;
    const point = mapPixelToScreen(worldToMapPixel(position.x, position.y));
    const yaw = quaternionYaw(state.pose.pose.orientation) - mapOriginYaw();
    context.save();
    context.translate(point.x, point.y);
    context.rotate(-yaw);
    context.beginPath();
    context.moveTo(14, 0);
    context.lineTo(-9, -9);
    context.lineTo(-5, 0);
    context.lineTo(-9, 9);
    context.closePath();
    context.fillStyle = "#087f73";
    context.strokeStyle = "#ffffff";
    context.lineWidth = 2;
    context.fill();
    context.stroke();
    context.restore();
  }

  function drawDirectionArrow(context, startWorld, endWorld, color, publishStyle) {
    const start = mapPixelToScreen(worldToMapPixel(startWorld.x, startWorld.y));
    const end = mapPixelToScreen(worldToMapPixel(endWorld.x, endWorld.y));
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    const length = Math.max(18, Math.hypot(dx, dy));
    const angle = Math.atan2(dy, dx);
    context.save();
    context.translate(start.x, start.y);
    context.rotate(angle);
    context.beginPath();
    context.moveTo(0, 0);
    context.lineTo(length, 0);
    context.strokeStyle = color;
    context.lineWidth = publishStyle ? 4 : 3;
    context.stroke();
    context.beginPath();
    context.moveTo(length, 0);
    context.lineTo(length - 11, -7);
    context.lineTo(length - 11, 7);
    context.closePath();
    context.fillStyle = color;
    context.fill();
    context.beginPath();
    context.arc(0, 0, 5, 0, Math.PI * 2);
    context.fillStyle = "#ffffff";
    context.fill();
    context.strokeStyle = color;
    context.lineWidth = 2;
    context.stroke();
    context.restore();
  }

  function worldToMapPixel(x, y) {
    const origin = state.map.origin.position;
    const yaw = mapOriginYaw();
    const dx = x - origin.x;
    const dy = y - origin.y;
    const localX = Math.cos(yaw) * dx + Math.sin(yaw) * dy;
    const localY = -Math.sin(yaw) * dx + Math.cos(yaw) * dy;
    return {
      x: localX / state.map.resolution,
      y: state.map.height - localY / state.map.resolution,
    };
  }

  function mapPixelToWorld(point) {
    const origin = state.map.origin.position;
    const yaw = mapOriginYaw();
    const localX = point.x * state.map.resolution;
    const localY = (state.map.height - point.y) * state.map.resolution;
    return {
      x: origin.x + Math.cos(yaw) * localX - Math.sin(yaw) * localY,
      y: origin.y + Math.sin(yaw) * localX + Math.cos(yaw) * localY,
    };
  }

  function mapPixelToScreen(point) {
    return {
      x: state.view.offsetX + point.x * state.view.scale,
      y: state.view.offsetY + point.y * state.view.scale,
    };
  }

  function screenToMapPixel(point) {
    return {
      x: (point.x - state.view.offsetX) / state.view.scale,
      y: (point.y - state.view.offsetY) / state.view.scale,
    };
  }

  function mapOriginYaw() {
    return quaternionYaw(state.map.origin.orientation);
  }

  function quaternionYaw(quaternion) {
    const siny = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
    const cosy = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    return Math.atan2(siny, cosy);
  }

  function zoomAt(factor, x, y) {
    if (!state.map) return;
    const oldScale = state.view.scale;
    const newScale = clamp(oldScale * factor, 0.05, 80);
    const mapX = (x - state.view.offsetX) / oldScale;
    const mapY = (y - state.view.offsetY) / oldScale;
    state.view.scale = newScale;
    state.view.offsetX = x - mapX * newScale;
    state.view.offsetY = y - mapY * newScale;
    renderMap();
  }

  function localPointer(event) {
    const rect = elements.canvas.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  function handleCanvasPointerDown(event) {
    if (!state.map) return;
    elements.canvas.setPointerCapture(event.pointerId);
    const point = localPointer(event);

    if (state.mapTool === "view") {
      state.viewPointers.set(event.pointerId, point);
      if (state.viewPointers.size === 1) {
        state.panLast = point;
      } else if (state.viewPointers.size === 2) {
        const points = Array.from(state.viewPointers.values());
        const midpoint = midpointOf(points[0], points[1]);
        state.pinch = {
          distance: distanceOf(points[0], points[1]),
          scale: state.view.scale,
          mapPoint: screenToMapPixel(midpoint),
        };
      }
      elements.canvas.classList.add("dragging");
      return;
    }

    if (baseMode() !== "navigation") {
      showToast("初始位姿和导航目标仅在导航模式可用", true);
      return;
    }
    state.actionPointer = event.pointerId;
    const world = mapPixelToWorld(screenToMapPixel(point));
    state.draft = { start: world, end: world };
    renderMap();
  }

  function handleCanvasPointerMove(event) {
    const point = localPointer(event);
    if (state.mapTool === "view" && state.viewPointers.has(event.pointerId)) {
      state.viewPointers.set(event.pointerId, point);
      if (state.viewPointers.size === 1 && state.panLast) {
        state.view.offsetX += point.x - state.panLast.x;
        state.view.offsetY += point.y - state.panLast.y;
        state.panLast = point;
      } else if (state.viewPointers.size >= 2 && state.pinch) {
        const points = Array.from(state.viewPointers.values()).slice(0, 2);
        const midpoint = midpointOf(points[0], points[1]);
        const factor = distanceOf(points[0], points[1]) / Math.max(1, state.pinch.distance);
        state.view.scale = clamp(state.pinch.scale * factor, 0.05, 80);
        state.view.offsetX = midpoint.x - state.pinch.mapPoint.x * state.view.scale;
        state.view.offsetY = midpoint.y - state.pinch.mapPoint.y * state.view.scale;
      }
      renderMap();
      return;
    }

    if (state.actionPointer === event.pointerId && state.draft) {
      state.draft.end = mapPixelToWorld(screenToMapPixel(point));
      renderMap();
    }
  }

  function handleCanvasPointerUp(event) {
    if (state.mapTool === "view") {
      state.viewPointers.delete(event.pointerId);
      state.pinch = null;
      const remaining = Array.from(state.viewPointers.values());
      state.panLast = remaining.length ? remaining[0] : null;
      if (!remaining.length) elements.canvas.classList.remove("dragging");
      return;
    }

    if (state.actionPointer !== event.pointerId || !state.draft) return;
    const draft = state.draft;
    state.actionPointer = null;
    state.draft = null;
    publishMapAction(draft);
    renderMap();
  }

  function publishMapAction(draft) {
    if (!state.interfaces || !state.connected) return;
    const dx = draft.end.x - draft.start.x;
    const dy = draft.end.y - draft.start.y;
    const yaw = Math.hypot(dx, dy) < 0.03 ? 0 : Math.atan2(dy, dx);
    const orientation = yawQuaternion(yaw);
    const stamp = rosTimestamp();

    if (state.mapTool === "pose") {
      const covariance = new Array(36).fill(0);
      covariance[0] = 0.25;
      covariance[7] = 0.25;
      covariance[35] = 0.12;
      state.interfaces.initialPose.publish(new ROSLIB.Message({
        header: { stamp, frame_id: "map" },
        pose: {
          pose: {
            position: { x: draft.start.x, y: draft.start.y, z: 0 },
            orientation,
          },
          covariance,
        },
      }));
      showToast("初始位姿已发送");
    } else if (state.mapTool === "goal") {
      state.interfaces.navigationGoal.publish(new ROSLIB.Message({
        header: { stamp, frame_id: "map" },
        pose: {
          position: { x: draft.start.x, y: draft.start.y, z: 0 },
          orientation,
        },
      }));
      showToast("导航目标已发送");
    }
  }

  function callTrigger(name) {
    return new Promise(function (resolve, reject) {
      if (!state.connected || !state.interfaces) {
        reject(new Error("ROS 尚未连接"));
        return;
      }
      const service = state.interfaces.services[name];
      if (!service) {
        reject(new Error("服务不存在：" + name));
        return;
      }
      let settled = false;
      const timeoutMs = name === "save_map" ? 35000 : 22000;
      const timeout = window.setTimeout(function () {
        if (!settled) {
          settled = true;
          reject(new Error("操作超时，请检查树莓派终端"));
        }
      }, timeoutMs);
      service.callService(
        new ROSLIB.ServiceRequest({}),
        function (result) {
          if (settled) return;
          settled = true;
          clearTimeout(timeout);
          if (result.success) resolve(result.message || "操作完成");
          else reject(new Error(result.message || "操作失败"));
        },
        function (error) {
          if (settled) return;
          settled = true;
          clearTimeout(timeout);
          reject(new Error(String(error)));
        }
      );
    });
  }

  async function runOperation(operation, pendingMessage) {
    if (state.busy) return;
    state.busy = true;
    updateControls();
    setSystemMessage(pendingMessage, false);
    try {
      const message = await operation();
      showToast(message);
    } catch (error) {
      showToast(error.message || String(error), true);
      setSystemMessage(error.message || String(error), true);
    } finally {
      state.busy = false;
      updateControls();
    }
  }

  function updateControls() {
    const mode = baseMode();
    elements.modeButtons.forEach(function (button) {
      button.disabled = !state.connected || state.busy;
    });
    elements.saveMap.disabled = !state.connected || state.busy || mode !== "mapping";
    elements.stopMode.disabled = !state.connected || state.busy;
    elements.emergencyStop.disabled = !state.connected;
    elements.mapToolButtons.forEach(function (button) {
      const requiresNavigation = button.dataset.mapTool !== "view";
      button.disabled = requiresNavigation && mode !== "navigation";
      if (button.disabled && button.classList.contains("active")) setMapTool("view");
    });
  }

  function baseMode() {
    return String(state.manager.mode || "stopped").replace("external_", "");
  }

  function setMapTool(tool) {
    state.mapTool = tool;
    state.draft = null;
    elements.canvas.dataset.tool = tool;
    elements.mapToolButtons.forEach(function (button) {
      button.classList.toggle("active", button.dataset.mapTool === tool);
    });
    renderMap();
  }

  function handleJoystickPointerDown(event) {
    if (!state.connected || !state.manager.nodes || !state.manager.nodes.base) {
      showToast("底盘节点尚未运行", true);
      return;
    }
    state.joystick.active = true;
    state.joystick.pointerId = event.pointerId;
    elements.joystick.setPointerCapture(event.pointerId);
    elements.joystick.classList.add("active");
    elements.driveState.textContent = "手动控制中";
    if (baseMode() === "navigation" && state.interfaces) {
      state.interfaces.navigationCancel.publish(new ROSLIB.Message({
        stamp: { secs: 0, nsecs: 0 }, id: "",
      }));
    }
    updateJoystick(event);
  }

  function handleJoystickPointerMove(event) {
    if (state.joystick.active && state.joystick.pointerId === event.pointerId) {
      updateJoystick(event);
    }
  }

  function updateJoystick(event) {
    const rect = elements.joystick.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;
    const knobRadius = elements.joystickKnob.offsetWidth / 2;
    const radius = Math.max(1, rect.width / 2 - knobRadius - 8);
    let dx = event.clientX - centerX;
    let dy = event.clientY - centerY;
    const distance = Math.hypot(dx, dy);
    if (distance > radius) {
      dx = dx / distance * radius;
      dy = dy / distance * radius;
    }

    const preset = speedPresets[state.speedPreset];
    state.joystick.linear = clamp(-dy / radius * preset.linear, -preset.linear, preset.linear);
    state.joystick.angular = clamp(-dx / radius * preset.angular, -preset.angular, preset.angular);
    elements.joystickKnob.style.transform = `translate(${dx}px, ${dy}px)`;
    updateVelocityReadout();
    publishManualCommand();
  }

  function releaseJoystick() {
    if (!state.joystick.active && state.joystick.linear === 0 && state.joystick.angular === 0) return;
    state.joystick.active = false;
    state.joystick.pointerId = null;
    state.joystick.linear = 0;
    state.joystick.angular = 0;
    elements.joystick.classList.remove("active");
    elements.joystickKnob.style.transform = "translate(0, 0)";
    elements.driveState.textContent = "待命";
    updateVelocityReadout();
    publishManualCommand();
    window.setTimeout(publishManualCommand, 80);
  }

  function publishManualCommand() {
    if (!state.connected || !state.interfaces) return;
    state.interfaces.manualCmd.publish(new ROSLIB.Message({
      linear: { x: state.joystick.linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: state.joystick.angular },
    }));
  }

  function updateVelocityReadout() {
    elements.linearValue.textContent = state.joystick.linear.toFixed(2);
    elements.angularValue.textContent = state.joystick.angular.toFixed(2);
  }

  function setSystemMessage(message, isError) {
    elements.systemMessage.textContent = message;
    elements.systemMessage.classList.toggle("error", Boolean(isError));
  }

  function showToast(message, isError) {
    clearTimeout(state.toastTimer);
    elements.toast.textContent = message;
    elements.toast.classList.toggle("error", Boolean(isError));
    elements.toast.classList.add("visible");
    state.toastTimer = window.setTimeout(function () {
      elements.toast.classList.remove("visible");
    }, 3200);
  }

  function yawQuaternion(yaw) {
    return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) };
  }

  function rosTimestamp() {
    const milliseconds = Date.now();
    return {
      secs: Math.floor(milliseconds / 1000),
      nsecs: (milliseconds % 1000) * 1000000,
    };
  }

  function midpointOf(a, b) {
    return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
  }

  function distanceOf(a, b) {
    return Math.hypot(a.x - b.x, a.y - b.y);
  }

  function clamp(value, minimum, maximum) {
    return Math.max(minimum, Math.min(maximum, value));
  }

  elements.modeButtons.forEach(function (button) {
    button.addEventListener("click", function () {
      const mode = button.dataset.startMode;
      runOperation(() => callTrigger(`start_${mode}`), `正在切换到${modeLabels[mode]}模式`);
    });
  });

  elements.stopMode.addEventListener("click", function () {
    runOperation(() => callTrigger("stop_mode"), "正在停止当前模式");
  });

  elements.saveMap.addEventListener("click", function () {
    if (typeof elements.saveDialog.showModal === "function") {
      elements.saveDialog.showModal();
    } else if (window.confirm("保存当前地图并更新默认导航地图？")) {
      runOperation(() => callTrigger("save_map"), "正在保存地图");
    }
  });

  elements.confirmSave.addEventListener("click", function () {
    window.setTimeout(function () {
      runOperation(() => callTrigger("save_map"), "正在保存地图");
    }, 0);
  });

  elements.emergencyStop.addEventListener("click", async function () {
    releaseJoystick();
    setSystemMessage("正在停车并取消导航", false);
    try {
      const message = await callTrigger("emergency_stop");
      showToast(message);
    } catch (error) {
      showToast(error.message || String(error), true);
      setSystemMessage(error.message || String(error), true);
    }
  });

  elements.mapToolButtons.forEach(function (button) {
    button.addEventListener("click", function () {
      if (!button.disabled) setMapTool(button.dataset.mapTool);
    });
  });

  elements.zoomIn.addEventListener("click", function () {
    const rect = elements.viewport.getBoundingClientRect();
    zoomAt(1.25, rect.width / 2, rect.height / 2);
  });
  elements.zoomOut.addEventListener("click", function () {
    const rect = elements.viewport.getBoundingClientRect();
    zoomAt(0.8, rect.width / 2, rect.height / 2);
  });
  elements.fitMap.addEventListener("click", fitMap);

  elements.canvas.addEventListener("wheel", function (event) {
    if (!state.map) return;
    event.preventDefault();
    const point = localPointer(event);
    zoomAt(event.deltaY < 0 ? 1.12 : 0.89, point.x, point.y);
  }, { passive: false });
  elements.canvas.addEventListener("pointerdown", handleCanvasPointerDown);
  elements.canvas.addEventListener("pointermove", handleCanvasPointerMove);
  elements.canvas.addEventListener("pointerup", handleCanvasPointerUp);
  elements.canvas.addEventListener("pointercancel", handleCanvasPointerUp);

  elements.speedButtons.forEach(function (button) {
    button.addEventListener("click", function () {
      state.speedPreset = button.dataset.speed;
      elements.speedButtons.forEach((item) => item.classList.toggle("active", item === button));
    });
  });

  elements.joystick.addEventListener("pointerdown", handleJoystickPointerDown);
  elements.joystick.addEventListener("pointermove", handleJoystickPointerMove);
  elements.joystick.addEventListener("pointerup", releaseJoystick);
  elements.joystick.addEventListener("pointercancel", releaseJoystick);
  window.addEventListener("blur", releaseJoystick);
  document.addEventListener("visibilitychange", function () {
    if (document.hidden) releaseJoystick();
  });

  window.setInterval(function () {
    if (state.joystick.active) publishManualCommand();
  }, 100);

  new ResizeObserver(resizeCanvas).observe(elements.viewport);
  setMapTool("view");
  resizeCanvas();
  updateControls();
  connectRos();
})();
