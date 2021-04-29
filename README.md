FTC
---
# 分支说明
1. 与main不同的功能
    使用`void positionUpdate()`计算实时位置坐标，原点(0,0)为赛场左下角点。
    使用`autoAim(TargetObject select)`对设定好的目标进行自动瞄准


# 程序文件说明
1. 自动部分
    AutoGyroWy.java
    **现在自动部分不归wy管,不好用与wy无关**

2. 手动部分
    manual_Iterative_Wy.java

# 按键及操作说明
1. 手柄按键
    1. gamePad1: 主要底盘操作
        1. 两个摇杆: 
            左边上下
            右边左右
        2. B
            回归绝对0°
        3. X
            重设初始坐标并初始化陀螺仪
        4. left&right trigger
            大转弯
        5. left&right bumper
            小转弯
        6. 上下左右按键
            上: 自动瞄准篮筐
            左: 自动瞄准第一个杆子
            下: 自动瞄准第二个杆子
            右: 自动瞄准第三个杆子

    2. gamePad2: 主要发射操作
        1. 左边摇杆
            收集圈
        2. Y
            升起电梯,准备发射
        3. A
            降下电梯
        4. right bump
            发射(只有当升起电梯时才有效)

2. 注意事项
    1. 手动开始后,需要将车开到特定位置,然后按下X进行初始对准

# 说明
附中仙林2021FTC比赛相关代码
