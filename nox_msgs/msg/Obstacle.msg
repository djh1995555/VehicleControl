Header header

uint64 id                       # 障碍物ID

Trajectory prediction           # 障碍物预测路径（静态障碍物为空，或只有一个点）

geometry_msgs/Pose pose         # 障碍物位姿

geometry_msgs/Vector3 length    # 障碍物包围在x, y, z方向的长度（该方向是障碍物本身的坐标系）

geometry_msgs/Polygon vertexes  # 障碍物顶点

geometry_msgs/Vector3 speed     # 障碍物速度，秒速

float64 confidence              # 障碍物可靠性

uint32 type                     # 障碍物类型

Overlap[] overlaps              # 障碍物的覆盖情况（给地图使用）

# -----------------------------------
# 障碍物类型枚举: TODO

uint32 UNKNOWN = 0
uint32 HUMAN = 1
uint32 CAR = 2

# -----------------------------------
