# Codecraft Huawei 2019


## 题设

* 道路进入规则:
    * 车道号小的优先于车道号大的
    * 直行优先于转弯, 左转优先于右转
    * 综上, 直行的小车道号依然大于转弯的大车道号
* 道路信息包括如下, 部分信息用于构图, 部分信息作为限制
    * 道路 id
    * 长度
    * 最高限速
    * 车道数
    * 起始点 id (路口 id)
    * 终点 id (路口 id)
    * 是否双向
* 车辆信息:
    * 车辆 id
    * 始发地 (路口 id)
    * 目的地 (路口 id)
    * 最高速度
    * 计划出发时间
* 路口信息:
    * 路口 id
    * 道路 id1, 道路 id2, 道路 id3, 道路 id4 (顺时针方向编排, 从地图的北开始, -1 表示无路可走)
* 地图, 以路口为顶点, 道路为边的有向连通图
* 路口连接的道路数不超过 4, 但算上车道数的话, 就可以多于 4 了
* 所有道路的长度都是整数, 且均不小于 6
* 车辆行驶的最高速度都是整数
* 车身长度固定为 1
* 行车路线给出的是车辆经过的道路, 不计较车道号, 但是检测时, 系统按给定的答案调度, 会考虑车道号
* 禁止掉头行驶
* 车辆均以可行进的最大车速前进, 不得主动降速行驶
* 车辆到达实际出发时间 (不是计划???), 需要上路行驶. 存在同时多辆到达出发时间且初始道路相同的情况, 按车辆编号由小到达上路.
* NOTE: 选手生成的时间是实际出发时间, 官方给定的 car.txt 中的是计划出发时间. 两者不一样, 评估时按照实际出发时间调度.
* 实际出发时间, 不得早于计划出发时间, 但可能晚于计划出发时间.
* 优先运行已经在路上运行的车辆, 再运行等待上路的车辆 (看起来是, 对于同一条道路, 有空位可进入的情况下, 路上的车优先于车库里的车)
* 如果车辆行进过程中不会通过路口, 即当前时刻过后依然停留在路口, 只影响当前车道排在其后的所有车辆的行进
* 通过路口时, 所有车辆同时通过, 不考虑在过路口的同一时刻内速度的差异.
* 车辆在即将进入的下一条道路上的行驶距离, 不超过其在下一条道路上的最大行驶速度在单位时间内行驶的距离
* 道路, 具体到车道, 不一定会被充满
* 车辆在下一条道路的行驶距离, 由单位时间内它在下一条道路上通过的最大距离-当前道路剩下的距离决定.
* 官方没有说明的是, 车辆在调度完成之后, 后面的车如何行进.
* 当前道路剩余距离大于单位时间内在下一条道路上能行驶的最大距离的, 不通过路口, 到达当前路口最前方位置.
* 从 0 时刻开始, 每个调度记为一个时间片, 一个调度驱动所有车辆行驶一个时间单位.
* 车辆的最小行驶距离为 1.
* 系统调度先调度路上的车, 当路上的车全部不可再行驶后再调度等待上路的车
* 等待上路的车, 按车辆 id 升序进行调度.
* 输出 `answer.txt` 文件, 格式如下:
    * 车辆 id
    * 实际出发时间 (可晚于计划出发时间)
    * 行驶路线序列 (道路的序列)
    * 一行一条数据, 格式: `(carId, StartTime, RoadId1, Road...)`
* 评分规则
    * 系统调度时间短者胜出 (从系统调度开始时间算, 而非第一辆车实际开始运行的时间, 至全部车辆全部到达目的地的总时间.)
    * 系统调度时间相同, 所有车辆的行驶用时总时间少者胜
    * 以上还是相同, 程序运行时间少者胜出
* 整个系统调度按路口ID升序进行调度各个路口,路口内各道路按道路ID升序进行调度
* 调度进入路口方向的道路


## 思路

### 零散的想法

* 这个问题, 可以将边转化为顶点来处理的样子.
* 可设置网络中的车辆上限, 或者局部网络的车辆上限 (超参数). 可以写一个 `release-cars` 函数, 以网络中的车数为参数, 释放上路车辆数.
* `drive-through-junction`, 重头戏啦. 可以像 AlphaGo 那样, 对驶入不同道路的结果做一个预判, 根据该值选择下一条道路
* 车辆可以带一个`剩余完成百分比`的属性, 作为路径选择的依据之一, 但是它并不完全以剩余距离数为依据, 起码还得考虑走剩余路径的最大速度. 感觉也不能完全以剩余预估时间为依据, 可能会增大吧?
* `drive`, 如果车辆在一个调度内只在同一条道路上行驶的话
* 随机延迟出发? 还是启发式延迟出发?
* 道路容量 = 车道数 * 道路长度
* 在进入道路的时候, 不同来源的车辆有不同的优先级, 其他时候, 不同道路互不干扰
* 一个时间片, 一辆车可能通过好几条道路
* 如果将速度为 1 视作基本情况, 其他的速度都是加速版
* 预先勾勒出道路的替代道路, 当优选道路不可用时, 切换到相邻道路去.
* 车辆到达目的地时, 后面车辆的运行状态的转变, 也许和过路口差不多.
* 用链表表示车道, 好处是, 快速获得前车信息, 链表的头作为路尾, 即进入下一条道路的路口那个位置
* 不允许掉头是一个很重要的限制
* 想 Git 一样要有一个临时状态, 用于计划, 最后 `apply` 再将计划施展到实际中
* 当车辆进入新的道路, 有哪些量发生了改变:
    * 车辆篇
        * 车辆的位置, start_cross_id 转为下一个路口;
        * 车辆的速度, current_speed 受前车/道路限速影响;
    * 道路篇
        * 前后两条道路的容量都会发生改变, 前一条道路的容量增加, 后一条道路的容量减少 (但这一点其实不必计算, 因为每次都是实时获取新的容量的)
            * 车辆从前一条车道上消失, 在下一条车道上出现
        * 前后两条道路的 weight 要被改变. (最简单的)策略不变的话, 还是取道路长度除以当前速度)
        * 前后两条道路的速度都会改变
            * 前一条车道的前置速度被改变, 后续所有车辆的速度都可以被改变
            * 后一条车道的速度会被降为车道速度和车辆速度中的小值