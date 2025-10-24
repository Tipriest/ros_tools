#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
from typing import Dict

import numpy as np
import rospy
from sensor_msgs.msg import JointState

try:
    import pinocchio as pin
    from pinocchio.visualize import MeshcatVisualizer
except Exception as e:
    rospy.logerr("Failed to import pinocchio/meshcat: %s", e)
    raise


class MeshcatRosViz:
    def __init__(self) -> None:
        # Params
        self.urdf_path = rospy.get_param(
            "~urdf_path",
            "/home/tipriest/Documents/legged_localization_benchmark/src/unitree_ros/robots/hexapod_description/elspider_mini_description/urdf/elsipder_mini.urdf",
        )
        # Mesh directory root used by Pinocchio to resolve package:// URIs
        self.mesh_dir = rospy.get_param(
            "~mesh_dir",
            "/home/tipriest/Documents/legged_localization_benchmark/src/unitree_ros/robots/hexapod_description/elspider_mini_description",
        )
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/elspider_mini/joint_states")
        self.free_flyer = rospy.get_param("~free_flyer", False)
        self.open_viewer = rospy.get_param("~open_viewer", True)

        if not os.path.isfile(self.urdf_path):
            rospy.logwarn("URDF file not found at '%s' — set ~urdf_path param.", self.urdf_path)

        # Build models
        if self.free_flyer:
            self.model, self.cmodel, self.vmodel = pin.buildModelsFromUrdf(
                self.urdf_path, self.mesh_dir, pin.JointModelFreeFlyer()
            )
        else:
            self.model, self.cmodel, self.vmodel = pin.buildModelsFromUrdf(
                self.urdf_path, self.mesh_dir
            )
        self.data = self.model.createData()

        # Visualizer
        self.viz = MeshcatVisualizer(self.model, self.cmodel, self.vmodel)
        try:
            self.viz.initViewer(open=self.open_viewer)
        except Exception as e:
            rospy.logerr("Failed to init Meshcat viewer: %s", e)
            raise
        self.viz.loadViewerModel()

        # Current configuration
        self.q = pin.neutral(self.model)

        # Map joint names -> (idx_q, nq)
        self.name_to_qslot: Dict[str, tuple] = {}
        for jid in range(1, self.model.njoints):  # skip root/universe
            name = self.model.names[jid]
            idx_q = int(self.model.idx_qs[jid])
            nq = int(self.model.joints[jid].nq())
            self.name_to_qslot[name] = (idx_q, nq)

        # Subscribe
        self.sub = rospy.Subscriber(self.joint_state_topic, JointState, self._on_joint_state, queue_size=10)

        # Display neutral at start
        self.viz.display(self.q)
        rospy.loginfo("meshcat_vis: ready (urdf=%s)", self.urdf_path)

    def _on_joint_state(self, msg: JointState):
        # Defensive checks
        if not msg.name or not msg.position:
            return
        # Update q for 1-DoF joints matching incoming names
        for name, pos in zip(msg.name, msg.position):
            slot = self.name_to_qslot.get(name)
            if slot is None:
                continue
            idx_q, nq = slot
            if nq == 1:
                try:
                    self.q[idx_q] = float(pos)
                except Exception:
                    pass
            else:
                # Multi-DoF joints would need vector segments; not expected here
                pass
        # Display updated configuration
        try:
            self.viz.display(self.q)
        except Exception as e:
            rospy.logwarn_throttle(1.0, "Meshcat display error: %s", e)


def main():
    rospy.init_node("meshcat_vis")
    try:
        MeshcatRosViz()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
