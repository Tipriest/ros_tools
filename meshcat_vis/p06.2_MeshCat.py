# 这个例子展示了如何在meshcat中加载和移动机器人。
# 注意：此功能需要安装Meshcat，可以通过 pip install --user meshcat 来安装。
 
import sys
from pathlib import Path
 
import numpy as np
import pinocchio as pin
# 导入 Meshcat 可视化工具
from pinocchio.visualize import MeshcatVisualizer
 
# --- 模型加载 ---
# 加载URDF模型。
pinocchio_model_dir = Path("/opt/openrobots/share") 
 
model_path = pinocchio_model_dir / "example-robot-data/robots"
mesh_dir = pinocchio_model_dir
urdf_filename = "solo.urdf"
urdf_model_path = model_path / "solo_description/robots" / urdf_filename
 
# 从URDF文件构建运动学、碰撞和可视化模型。
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
)
 
# --- 启动 MeshCat ---
# 启动一个新的MeshCat服务器和客户端。
# 注意：服务器也可以通过在终端中运行 "meshcat-server" 命令来单独启动，
# 这样可以在当前脚本结束后保持服务器活动。
#
# open=True 选项会打开可视化器窗口。
# 注意：也可以通过访问提供的URL来单独打开可视化器。
try:
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. "
        "It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)
 
# 在查看器中加载机器人模型。
viz.loadViewerModel()
 
# --- 显示机器人和几何体 ---
# 显示一个机器人位形。
q0 = pin.neutral(model)
viz.display(q0)
viz.displayVisuals(True) # 确保可视化模型是可见的
 
# 从solo机器人的主体验创建一个凸包形状。
mesh = visual_model.geometryObjects[0].geometry
mesh.buildConvexRepresentation(True)
convex = mesh.convex
 
# 在场景中放置凸包对象并显示它。
if convex is not None:
    placement = pin.SE3.Identity()
    placement.translation[0] = 2.0
    geometry = pin.GeometryObject("convex", 0, placement, convex)
    geometry.meshColor = np.ones(4)
    # 为凸包对象添加Phong材质。
    geometry.overrideMaterial = True
    geometry.meshMaterial = pin.GeometryPhongMaterial()
    geometry.meshMaterial.meshEmissionColor = np.array([1.0, 0.1, 0.1, 1.0])
    geometry.meshMaterial.meshSpecularColor = np.array([0.1, 1.0, 0.1, 1.0])
    geometry.meshMaterial.meshShininess = 0.8
    visual_model.addGeometryObject(geometry)
    # 修改visual_model后，我们必须在可视化器中重建相关数据。
    viz.rebuildData()
 
# 显示另一个机器人。
viz2 = MeshcatVisualizer(model, collision_model, visual_model)
viz2.initViewer(viz.viewer)
viz2.loadViewerModel(rootNodeName="pinocchio2")
q = q0.copy()
q[1] = 1.0
viz2.display(q)
 
# --- 模拟和动画 ---
# 站立位形
q1 = np.array(
    [0.0, 0.0, 0.235, 0.0, 0.0, 0.0, 1.0, 0.8, -1.6, 0.8, -1.6, -0.8, 1.6, -0.8, 1.6]
)
 
v0 = np.random.randn(model.nv) * 2
data = viz.data
pin.forwardKinematics(model, data, q1, v0)
frame_id = model.getFrameId("HR_FOOT")
viz.display()
# 绘制指定坐标系的速度矢量。
viz.drawFrameVelocities(frame_id=frame_id)
 
model.gravity.linear[:] = 0.0
dt = 0.01
 
# 定义一个简单的模拟循环。
def sim_loop():
    tau0 = np.zeros(model.nv)
    qs = [q1]
    vs = [v0]
    nsteps = 100
    for i in range(nsteps):
        q = qs[i]
        v = vs[i]
        # 使用ABA算法（Articulated-Body Algorithm）计算正向动力学。
        a1 = pin.aba(model, data, q, v, tau0)
        vnext = v + dt * a1
        qnext = pin.integrate(model, q, dt * vnext)
        qs.append(qnext)
        vs.append(vnext)
        viz.display(qnext)
        viz.drawFrameVelocities(frame_id=frame_id)
    return qs, vs
 
 
qs, vs = sim_loop()
 
fid2 = model.getFrameId("FL_FOOT")
 
# 定义一个回调函数，用于在播放动画的每一帧调用。
def my_callback(i, *args):
    viz.drawFrameVelocities(frame_id)
    viz.drawFrameVelocities(fid2)
 
# 创建一个视频录制上下文，并将动画保存到文件中。
with viz.create_video_ctx("../leap.mp4"):
    # 播放位形序列qs，时间步长为dt，并在每一帧调用回调函数。
    viz.play(qs, dt, callback=my_callback)
