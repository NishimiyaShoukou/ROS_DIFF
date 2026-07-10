#!/usr/bin/env python3

import json
import os
import signal
import subprocess
import threading
import time
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import rosnode
import rospkg
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


class StaticRequestHandler(SimpleHTTPRequestHandler):
    extensions_map = dict(SimpleHTTPRequestHandler.extensions_map)
    extensions_map.update({
        ".css": "text/css; charset=utf-8",
        ".html": "text/html; charset=utf-8",
        ".js": "application/javascript; charset=utf-8",
        ".json": "application/json; charset=utf-8",
    })

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; "
            "connect-src ws: wss:; img-src 'self' data:; object-src 'none'; "
            "base-uri 'none'; frame-ancestors 'none'",
        )
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        super().end_headers()

    def list_directory(self, path):
        self.send_error(404, "Not found")
        return None

    def log_message(self, format_string, *args):
        rospy.logdebug("web: " + format_string, *args)


class MobileRobotManager:
    MODES = ("idle", "mapping", "navigation")

    def __init__(self):
        self._lock = threading.RLock()
        self._child = None
        self._mode = "stopped"
        self._last_error = ""
        self._shutting_down = False
        self._manual_cmd = Twist()
        self._navigation_cmd = Twist()
        self._last_manual_cmd_time = rospy.Time(0)
        self._last_navigation_cmd_time = rospy.Time(0)

        self._http_port = int(rospy.get_param("~http_port", 8080))
        self._websocket_port = int(rospy.get_param("~websocket_port", 9090))
        self._auto_start_mode = rospy.get_param("~auto_start_mode", "idle")
        self._serial_port = rospy.get_param("~serial_port", "/dev/chassis")
        self._lidar_port = rospy.get_param("~lidar_port", "/dev/ydlidar")
        self._map_file = os.path.abspath(rospy.get_param("~map_file"))
        self._map_prefix = os.path.abspath(rospy.get_param("~map_prefix"))
        self._use_imu_yaw = bool(rospy.get_param("~use_imu_yaw", False))
        self._use_ekf = bool(rospy.get_param("~use_ekf", False))
        self._base_frame = rospy.get_param("~base_frame", "base_footprint")

        package_path = rospkg.RosPack().get_path("myrobot_web")
        self._web_root = os.path.join(package_path, "web")

        self._status_pub = rospy.Publisher(
            "/mobile/status", String, queue_size=1, latch=True
        )
        self._pose_pub = rospy.Publisher(
            "/mobile/robot_pose", PoseStamped, queue_size=1
        )
        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        rospy.Subscriber(
            "/mobile/cmd_vel", Twist, self._manual_cmd_callback, queue_size=1
        )
        rospy.Subscriber(
            "/mobile/nav_cmd_vel", Twist, self._navigation_cmd_callback, queue_size=1
        )

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        rospy.Service("/mobile/start_idle", Trigger, self._start_idle)
        rospy.Service("/mobile/start_mapping", Trigger, self._start_mapping)
        rospy.Service("/mobile/start_navigation", Trigger, self._start_navigation)
        rospy.Service("/mobile/stop_mode", Trigger, self._stop_mode)
        rospy.Service("/mobile/save_map", Trigger, self._save_map)
        rospy.Service("/mobile/emergency_stop", Trigger, self._emergency_stop)

        self._http_server = self._start_http_server()
        rospy.Timer(rospy.Duration(0.5), self._publish_status)
        rospy.Timer(rospy.Duration(0.2), self._publish_robot_pose)
        rospy.Timer(rospy.Duration(0.05), self._publish_muxed_command)
        rospy.Timer(rospy.Duration(1.0), self._auto_start, oneshot=True)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "Mobile control page: http://<raspberry-pi-ip>:%d", self._http_port
        )

    def _start_http_server(self):
        handler = partial(StaticRequestHandler, directory=self._web_root)
        server = ThreadingHTTPServer(("0.0.0.0", self._http_port), handler)
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, name="mobile-http")
        thread.daemon = True
        thread.start()
        return server

    def _auto_start(self, _event):
        if self._auto_start_mode in self.MODES:
            success, message = self._switch_mode(self._auto_start_mode)
            if success:
                rospy.loginfo(message)
            else:
                rospy.logwarn(message)
        elif self._auto_start_mode not in ("", "none", "stopped"):
            rospy.logwarn("Ignoring unknown auto_start_mode: %s", self._auto_start_mode)

    def _start_idle(self, _request):
        return self._trigger_mode("idle")

    def _start_mapping(self, _request):
        return self._trigger_mode("mapping")

    def _start_navigation(self, _request):
        return self._trigger_mode("navigation")

    def _trigger_mode(self, mode):
        success, message = self._switch_mode(mode)
        return TriggerResponse(success=success, message=message)

    def _switch_mode(self, mode):
        with self._lock:
            if mode not in self.MODES:
                return False, "Unknown robot mode: " + mode

            if self._child is not None and self._child.poll() is None:
                if self._mode == mode:
                    return True, mode + " is already running"
                self._stop_child_locked()
            else:
                external_mode = self._detect_external_mode()
                if external_mode is not None:
                    return (
                        False,
                        "Detected externally started ROS nodes (%s); stop their roslaunch first"
                        % external_mode,
                    )

            if mode == "navigation" and not os.path.isfile(self._map_file):
                return False, "Map file not found: " + self._map_file

            command = self._launch_command(mode)
            try:
                self._child = subprocess.Popen(command, start_new_session=True)
            except OSError as exc:
                self._last_error = str(exc)
                return False, "Failed to start %s: %s" % (mode, exc)

            self._mode = mode
            self._last_error = ""
            return True, "Started " + mode

    def _launch_command(self, mode):
        common = [
            "serial_port:=" + self._serial_port,
            "lidar_port:=" + self._lidar_port,
            "use_imu_yaw:=" + str(self._use_imu_yaw).lower(),
            "use_ekf:=" + str(self._use_ekf).lower(),
        ]
        if mode == "idle":
            return [
                "roslaunch",
                "myrobot",
                "bringup.launch",
                "enable_lidar:=true",
            ] + common
        if mode == "mapping":
            return [
                "roslaunch",
                "myrobot",
                "mapping_v2.launch",
                "rviz:=false",
            ] + common
        return [
            "roslaunch",
            "myrobot",
            "navigation_v2.launch",
            "rviz:=false",
            "map_file:=" + self._map_file,
            "cmd_vel_topic:=/mobile/nav_cmd_vel",
        ] + common

    def _stop_mode(self, _request):
        with self._lock:
            if self._child is None or self._child.poll() is not None:
                external_mode = self._detect_external_mode()
                if external_mode is not None:
                    return TriggerResponse(
                        success=False,
                        message="Mode is externally managed; stop its roslaunch terminal",
                    )
                self._mode = "stopped"
                self._publish_stop()
                return TriggerResponse(success=True, message="Robot is already stopped")

            self._stop_child_locked()
            return TriggerResponse(success=True, message="Stopped managed ROS mode")

    def _stop_child_locked(self):
        self._publish_stop()
        child = self._child
        self._child = None
        self._mode = "stopped"
        if child is None or child.poll() is not None:
            return

        signals = (
            (signal.SIGINT, 8.0, "SIGINT"),
            (signal.SIGTERM, 3.0, "SIGTERM"),
            (signal.SIGKILL, 2.0, "SIGKILL"),
        )
        for signum, timeout, name in signals:
            if child.poll() is not None:
                return
            try:
                os.killpg(child.pid, signum)
            except OSError as exc:
                rospy.logwarn("Failed to send %s to managed roslaunch: %s", name, exc)
                return
            try:
                child.wait(timeout=timeout)
                return
            except subprocess.TimeoutExpired:
                rospy.logwarn("Managed roslaunch did not stop after %s", name)

    def _save_map(self, _request):
        with self._lock:
            node_names = self._node_names()
            if "/slam_gmapping" not in node_names:
                return TriggerResponse(
                    success=False, message="Mapping is not running; no live map to save"
                )
            map_prefix = self._map_prefix

        map_directory = os.path.dirname(map_prefix)
        map_name = os.path.basename(map_prefix)
        try:
            os.makedirs(map_directory, exist_ok=True)
            result = subprocess.run(
                ["rosrun", "map_server", "map_saver", "-f", map_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=30.0,
                check=False,
                cwd=map_directory,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            with self._lock:
                self._last_error = str(exc)
            return TriggerResponse(success=False, message="Map save failed: " + str(exc))

        yaml_path = map_prefix + ".yaml"
        pgm_path = map_prefix + ".pgm"
        if result.returncode != 0 or not os.path.isfile(yaml_path) or not os.path.isfile(pgm_path):
            output = result.stdout.strip()[-300:]
            with self._lock:
                self._last_error = output or "map_saver returned %d" % result.returncode
                error_message = self._last_error
            return TriggerResponse(success=False, message="Map save failed: " + error_message)

        with self._lock:
            self._last_error = ""
        return TriggerResponse(success=True, message="Map saved to " + yaml_path)

    def _emergency_stop(self, _request):
        self._last_manual_cmd_time = rospy.Time(0)
        self._last_navigation_cmd_time = rospy.Time(0)
        self._publish_stop()
        cancel = GoalID()
        cancel.stamp = rospy.Time()
        cancel.id = ""
        self._cancel_pub.publish(cancel)
        return TriggerResponse(success=True, message="Velocity stopped and navigation cancelled")

    def _publish_stop(self):
        stop = Twist()
        for _ in range(3):
            self._cmd_pub.publish(stop)
            time.sleep(0.02)

    def _manual_cmd_callback(self, message):
        with self._lock:
            self._manual_cmd = message
            self._last_manual_cmd_time = rospy.Time.now()

    def _navigation_cmd_callback(self, message):
        with self._lock:
            self._navigation_cmd = message
            self._last_navigation_cmd_time = rospy.Time.now()

    def _publish_muxed_command(self, _event):
        with self._lock:
            now = rospy.Time.now()
            manual_fresh = (
                not self._last_manual_cmd_time.is_zero()
                and (now - self._last_manual_cmd_time).to_sec() <= 0.30
            )
            navigation_fresh = (
                not self._last_navigation_cmd_time.is_zero()
                and (now - self._last_navigation_cmd_time).to_sec() <= 0.30
            )

            if manual_fresh:
                command = self._manual_cmd
            elif navigation_fresh and self._mode == "navigation":
                command = self._navigation_cmd
            else:
                return
            self._cmd_pub.publish(command)

    def _detect_external_mode(self):
        names = self._node_names()
        if "/slam_gmapping" in names:
            return "mapping"
        if "/move_base" in names or "/amcl" in names:
            return "navigation"
        if "/base_controller" in names or "/ydlidar" in names:
            return "idle"
        return None

    @staticmethod
    def _node_names():
        try:
            return set(rosnode.get_node_names())
        except rosnode.ROSNodeIOException:
            return set()

    def _publish_status(self, _event):
        with self._lock:
            if self._child is not None:
                return_code = self._child.poll()
                if return_code is not None:
                    failed_mode = self._mode
                    self._child = None
                    self._mode = "stopped"
                    self._last_error = "%s roslaunch exited with code %d" % (
                        failed_mode,
                        return_code,
                    )

            names = self._node_names()
            external_mode = None
            if self._child is None:
                external_mode = self._detect_external_mode()
            displayed_mode = self._mode
            if external_mode is not None:
                displayed_mode = "external_" + external_mode

            status = {
                "mode": displayed_mode,
                "managed": self._child is not None,
                "map_available": os.path.isfile(self._map_file),
                "websocket_port": self._websocket_port,
                "nodes": {
                    "base": "/base_controller" in names,
                    "lidar": "/ydlidar" in names,
                    "mapping": "/slam_gmapping" in names,
                    "localization": "/amcl" in names,
                    "navigation": "/move_base" in names,
                },
                "error": self._last_error,
            }
            self._status_pub.publish(String(data=json.dumps(status, ensure_ascii=True)))

    def _publish_robot_pose(self, _event):
        for frame in ("map", "odom"):
            try:
                transform = self._tf_buffer.lookup_transform(
                    frame, self._base_frame, rospy.Time(0), rospy.Duration(0.02)
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                continue

            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            self._pose_pub.publish(pose)
            return

    def shutdown(self):
        with self._lock:
            if self._shutting_down:
                return
            self._shutting_down = True
            if self._child is not None and self._child.poll() is None:
                self._stop_child_locked()

        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()


def main():
    rospy.init_node("mobile_robot_manager")
    MobileRobotManager()
    rospy.spin()


if __name__ == "__main__":
    main()
