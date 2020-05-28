# 提取工具 extract-tools

提取rosbag中的topic信息，保存成TUM dataset的格式，方便进行离线运行。

Tools to extract rosbag topics to TUM style.

主目录下的convertRaw为NGC中转换为数据集的代码，供修改参考。

---

* 对depth图进行注册到红外上







---

当前为可以提取RGBD&红外信息的ready-to-go版本。

注意事项：

* 使用前修改launch文件里的topic和folder信息
* 基于ros，catkin_make后使用
* 需要手动建立文件夹。主文件夹为launch文件中的folder位置，主文件夹内建立/thermal, /rgb, /depth等三个子文件夹!!如果没有建立，不会报错，直接没有提取结果。



---

TODO：

1. 增加多相机支持
2. 增加IMU信息
3. 增加ground truth信息
4. 优化同步？（目前使用approximate time）

