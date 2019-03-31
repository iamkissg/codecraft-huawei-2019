import os
import sys
import logging
import copy
import random
from math import floor
from collections import OrderedDict, defaultdict
from queue import Queue

import numpy as np

import networkx as nx
from utc.road import DRIVEIN_ABLE, BLOCKED, TO_BE_SCHEDULED
from utc.car import CAR_TO_RUN, CAR_RUNNING, CAR_STOP, CAR_END


logger = logging.getLogger()


# TODO: 1. 所有的检查项都注释掉

class Scheduler(object):

    def __init__(self, crosses, roads, cars, capacity_threshold=0.9, num_cars_on_road=128):
        """根据路口和道路, 保存了几乎所有的静态量.
        路口肯定是不变的, 道路的长度, 限速都是不变的, 变化的包括:
            * 每条车道上的车辆数, 决定了可进入的车辆数
            * 每条车道上最后一辆车的速度, 决定了可进入的车速
        """

        self.roadnet = nx.DiGraph()
        self.roadnet.add_nodes_from([cross.cross_id for cross in crosses])
        self.roadnet.add_edges_from([
            (road.start_cross_id, road.end_cross_id, {'weight': road.length / road.highest_speed})
            for road in roads
        ])
        self.cars_to_run = cars   # 待上路车辆
        self.running_cars = []    # 路上车辆
        self.ended_cars = []      # 结束车辆
        self.crosses = OrderedDict([
            (cross.cross_id, cross)
            for cross in sorted(crosses, key=lambda c: int(c.cross_id))
        ])
        self.roads = OrderedDict([
            (road.road_id, road)
            for road in sorted(roads, key=lambda r: int(r.road_id[:r.road_id.find('#')]))
        ])
        # 根据两端的路口确定道路, 路口顺序不可颠倒
        self.cross_pair_to_road = {(road.start_cross_id, road.end_cross_id): road for road in self.roads.values()}

        # 确定进入路口方向的道路
        # 因为我们为每一条道路都编了编号, 无论是单项还是双向车道
        for cross in self.crosses.values():
            cross.connect_with_roads(self.roads)

        self.current_time = 0
        self.max_roadnet_capacity = sum([road.max_capacity for road in self.roads.values()])
        self.block_roadnet_capacity = floor(self.max_roadnet_capacity * capacity_threshold)
        self.num_cars_on_road = num_cars_on_road

        self._arrange_cars_to_run()

    def _send_run_signals(self):
        for car in self.running_cars:
            car.state = CAR_TO_RUN
            car.pass_intention = None
            car.road_to_turn = None
    
    def _check_positions(self, cars):
        for car in cars:
            if car.state == CAR_END:
                continue
            road = self.roads.get(car.on_road)
            lane = road.lanes[int(car.on_lane[car.on_lane.find('@')+1:])-1]
            assert lane.positions[car.on_position] == car, '{}, {} vs {}'.format(car.on_position, lane.positions[car.on_position], car)

    def schedule(self):
        # step1, 调度路上车辆
        self._send_run_signals()
        self._schedule_running_cars()
        self._check_positions(self.running_cars)

        # step3, 根据当前路网与道路的情况, 选择车辆上路
        self._schedule_cars_to_run()

        logger.info('第{t}个时间片调度\t调度完成'.format(t=self.current_time))
        self.current_time += 1

    def _get_road2car_flows(self, cross):
        """路口内的调度的限制
        1. 按照道路编号由小到达排序,
        2. 直行优先, 左转其次, 右转最次
        3. 每条道路内, 又车道优先级以及按照距离路口的距离安排行车

        本方法属于偷懒的办法, 为每条道路安排好直行左转右转的道路, 同时编排号道路内行车顺序
        """
        road2car_flows = {}  # [[当前道路], [右侧道路], [对面道路], [左边道路]] 视为一个 car_flow
        roads = list(cross.connected_roads.values())
        for i, road_id in enumerate(cross.connected_roads.keys()):
            cars = [
                roads[i%4].lanes[lane].positions[pos]
                for pos in range(roads[i%4].length)
                for lane in range(len(roads[i%4].lanes))
                if roads[i%4].lanes[lane].positions[pos] and roads[i%4].lanes[lane].positions[pos].state != CAR_STOP
            ] if roads[i%4] else []
            right_cars = [
                roads[(i+1)%4].lanes[lane].positions[pos]
                for pos in range(roads[(i+1)%4].length)
                for lane in range(len(roads[(i+1)%4].lanes))
                if roads[(i+1)%4].lanes[lane].positions[pos] and roads[(i+1)%4].lanes[lane].positions[pos].state != CAR_STOP
            ] if roads[(i+1)%4] else []
            opposite_cars = [
                roads[(i+2)%4].lanes[lane].positions[pos]
                for pos in range(roads[(i+2)%4].length)
                for lane in range(len(roads[(i+2)%4].lanes))
                if roads[(i+2)%4].lanes[lane].positions[pos] and roads[(i+2)%4].lanes[lane].positions[pos].state != CAR_STOP
            ] if roads[(i+2)%4] else []
            left_cars = [
                roads[(i+3)%4].lanes[lane].positions[pos]
                for pos in range(roads[(i+3)%4].length)
                for lane in range(len(roads[(i+3)%4].lanes))
                if roads[(i+3)%4].lanes[lane].positions[pos] and roads[(i+3)%4].lanes[lane].positions[pos].state != CAR_STOP
            ] if roads[(i+3)%4] else []

            road2car_flows[road_id] = [cars, right_cars, opposite_cars, left_cars]

        return road2car_flows

    def _schedule_running_cars(self):
        """对每条道路上的车辆, 按照从路口开始的顺序调度"""
        if len(self.running_cars) == 0:
            return

        while True:
            # 除了完成调度的车辆, 只有当全部车辆转为 CAR_STOP 才意味着本次调度结束
            if len([car for car in self.running_cars if car.state!=CAR_STOP]) == 0:
                break

            assert None not in self.running_cars

            # 9. 系统调度详细说明 6. 调度处理逻辑 第一步: 处理所有道路的车辆的顺序, 即让能跑的车都跑了
            # 以下处理会有一些问题, 因为车辆在 self.running_cars 中并非按照道路上的词序排列
            # self._schedule_cars_move_on_the_same_way(self.running_cars)
            for cross in self.crosses.values():
                for road in cross.connected_roads.values():
                    if road:
                        for lane in road.lanes:
                            self._schedule_cars_move_on_the_same_way([car for car in lane.positions if car])
                #         car_flows.extend([
                #             road.lanes[lane].positions[pos]
                #             for lane in range(len(road.lanes))
                #             for pos in range(road.length)
                #             if road.lanes[lane].positions[pos] # and road.lanes[lane].positions[pos].state != CAR_TO_RUN
                #         ])
                # self._schedule_cars_move_on_the_same_way(car_flows)

            # 经过以上调度, 路上只有两种状态的车辆, 已完成调度的 CAT_STOP 或等待调度的 CAR_RUNNING
            assert not [car for car in self.running_cars if car.state == CAR_TO_RUN]

            # logger.info('第{t}个时间片调度\t完成路网中所有可直接调度车辆, 剩余{n}辆车等待调度'.format(
            #     t=self.current_time, n=len([car.car_id for car in self.running_cars if car.state != CAR_STOP])))
            
            for cross in self.crosses.values():
                road2car_flows = self._get_road2car_flows(cross)
                # num_rest_cars 用以观察记录当前是否已经不存在可调度的车辆了, 也许路口内车辆在等待其他路口的车辆, 因此跳过
                num_rest_cars = len([car.car_id for car in self.running_cars if car.state != CAR_STOP])
                while True:
                    for road_id in sorted(cross.connected_roads.keys(), key=lambda k: int(k)):
                        if road_id in (-1, -2):
                            continue
                        cars, right_cars, opposite_cars, left_cars = road2car_flows[road_id]
                        # car_flows = cars + right_cars + opposite_cars + left_cars
                        # 名义上的过路口车辆调度, 但车辆可能过不了路口
                        self._schedule_cars_pass_cross(cars, right_cars, opposite_cars, left_cars)
                        # logger.info('\t\t剩余待调度车辆: {}'.format(len([car.car_id for car in self.running_cars if car.state != CAR_STOP])))
                        
                    # 条件成立, 路口内有车辆被调度, 也许还有车辆因为这一次调度可以走了, 因此不跳出 while 循环
                    new_num_rest_cars = len([car.car_id for car in self.running_cars if car.state != CAR_STOP])
                    if new_num_rest_cars < num_rest_cars:
                        num_rest_cars = new_num_rest_cars
                    else:
                        self._check_positions(self.running_cars)  # 不放心可以再检查一遍
                        break

        for car in self.running_cars:
            assert car.state == CAR_STOP

        # 当车道的最后一辆车发生变更时, 自动更新对应道路的权重, 因此不必多虑
        # 4. 路网要变
            # 1. 最重要的, 对应道路的权重要变 (在对整条道路进行调度后调整)

    def _schedule_cars_pass_cross(self, cars, right_cars, opposite_cars, left_cars):
        """偷懒起见, 很多和 _schedule_cars_move_on_the_same_way 有大量重复代码"""
        for car in cars:
            assert car
            if car.state == CAR_STOP or car.state == CAR_END:
                continue

            cross = self.crosses.get(car.start_cross_id)
            road = self.roads.get(car.on_road)
            lane = road.lanes[int(car.on_lane[car.on_lane.find('@')+1:])-1]
            road_to_turn = self.roads.get(car.road_to_turn)

            if lane.find_previous_car_position(car.on_position) is None:
                previous_car = None
            else:
                previous_car = lane.positions[lane.find_previous_car_position(car.on_position)]

            # 检查前车的状态, 有 3 种情况:
                # 1. 无前车状态
                    # 1.1. 可能在当前道路上继续前行, 可完成本次调度
                    # 1.2. 可能经过路口
                        # 1.2.1. 到达目的地
                        # 1.2.2 由于前方道路限制, 当前车辆停在路口位置, 并不进入下一条道路
                        # 1.2.3. 转上另一条路, 继续前行
                            # 1.2.3.1 前方道路都禁止进入, 继续等待
                            # 1.2.3.2 前方有道路可通行
                # 2. 前车调度完成, 在当前道路上继续前行, 完成本次调度
                    # 1. 会达到跟车状态, 唯一的区别是车速的变更
                    # 2. 不会达到跟车状态
                # 3. 前车处于等待调度状态

            # 1. 无前车
            if previous_car == None:
                assert lane.get_previous_car_current_speed(car.on_position) == lane.speed, '{} vs {}'.format(
                    lane.get_previous_car_current_speed(car.on_position), lane.speed)

                # 1.1. 在当前道路上继续前行, 完成本次调度
                if car.on_position > car.current_speed:  # 车辆不足以穿过路口
                    # logger.info('1 {} 无前车, 车辆距离路口: {}, 当前车速为: {}, (道路限速: {}, 车辆最大速度: {}), 因此不足以通过路口'.format(car.car_id, car.on_position, car.current_speed, lane.speed, car.highest_speed))
                    self._car_moves_on_the_same_way(car, lane, car.current_speed, is_following=True)
                    car.state = CAR_STOP
                    continue
                
                if car.on_position == car.current_speed:
                    self._car_moves_on_the_same_way(car, lane, car.on_position, is_following=True)
                    car.state = CAR_STOP
                    continue

                # 1.2 经过路口的
                elif car.current_speed > car.on_position:

                    # 1.2.1 到达目的地
                    if car.start_cross_id == car.end_cross_id:
                        lane.positions[car.on_position] = None
                        # 以下两个 on_变量不重要
                        # car.on_road = None
                        # car.on_lane = None
                        # car.on_position = None
                        car.on_road = None
                        car.on_lane = None
                        car.on_position = None
                        car.state = CAR_END
                        self.running_cars.remove(car)
                        self.ended_cars.append(car)

                        # 车辆离开车道, 车道可以变得空旷, 原先道路的状态势必改变, roadnet 的权只是可能改变
                        self._update_road_weight(road)
                        continue

                    # 1.2.2 由于前方道路限制, 当前车辆停在路口位置, 并不进入下一条道路
                    # 路口被封锁, 即与路口相连的其他道路都没有可进入的容量, 且所有末位的车都已经被调度过了
                    if self.get_cross_states_where_car_is(car) == BLOCKED:
                        self._car_moves_on_the_same_way(car, lane, car.on_position, is_following=True)
                        car.state = CAR_STOP
                        continue

                    if car.pass_intention == 'go_straight':
                        self._car_pass_cross(car, road_to_turn)
                        car.state = CAR_STOP
                        continue
                    elif car.pass_intention == 'turn_left':
                        if right_cars and right_cars[0].pass_intention == 'go_strainght':
                            return
                        self._car_pass_cross(car, road_to_turn)
                        car.state = CAR_STOP
                        continue
                    elif car.pass_intention == 'turn_right':
                        if left_cars and left_cars[0].pass_intention == 'go_straight':
                            return
                        if opposite_cars and opposite_cars[0].pass_intention == 'turn_left':
                            return
                        self._car_pass_cross(car, road_to_turn)
                        car.state = CAR_STOP
                        continue

            # 2. 前车的调度已经完成, 放心地往前开就是
            elif previous_car.state == CAR_STOP:  
                previous_car_position = previous_car.on_position
                distance_from_previous_car = car.on_position - previous_car_position
                # logger.info('2. {} 前车已完成调度, 距离前车: {}, 当前车速为: {}'.format(car.car_id, distance_from_previous_car, car.current_speed))

                # 车辆以单位时间内可前进的距离前进, 不会到达跟车状态
                if distance_from_previous_car > car.current_speed:
                    self._car_moves_on_the_same_way(car, lane, car.current_speed, is_following=False)
                # 车辆以单位时间内可前进的距离前进, 达到跟车状态
                elif distance_from_previous_car == car.current_speed:
                    self._car_moves_on_the_same_way(car, lane, car.current_speed-1, is_following=True)
                # 当前道路上的可前进距离小于车辆单位时间能可行进的距离, 车辆可能穿过路口
                car.state = CAR_STOP

            # 3. 前车处理等待调度状态
            elif previous_car.state == CAR_RUNNING:
                if not car.pass_intention:
                    self._make_plan_for_running_car(car)
                    # self._choose_a_road_to_turn(car)
                car.state = CAR_RUNNING
                continue
            else:
                raise RuntimeError('迷路了吧')

    def _schedule_cars_move_on_the_same_way(self, cars):
        """
        调度只可能在本车道上前行的车辆, 其他车辆标记为待调度
        (本方法与上一个方法有大量重复, 请参考上面的注释)

        # 车辆调度, 变更量检查
        # 1. 车要变
            # 1. 车速可能会变 (current_speed)
            # 4. 后车的车速可能也要变
            # 2. 车的起始路口要变 (start_cross_id)
            # 3. 车绑定的道路/车道/位置信息都要变 (on_road, on_lane, on_position)
        # 2. 车道要变
            # 1. 车道的某个位置要被车辆填充 (positions)
        """

        for car in cars:
            if car.state == CAR_STOP or car.state == CAR_END:
                continue

            road = self.roads.get(car.on_road)
            lane = road.lanes[int(car.on_lane[car.on_lane.find('@')+1:])-1]

            if lane.find_previous_car_position(car.on_position) is None:
                previous_car = None
            else:
                previous_car = lane.positions[lane.find_previous_car_position(car.on_position)]

            # 1. 无前车状态
            if previous_car == None:
                assert lane.get_previous_car_current_speed(car.on_position) == lane.speed, '{} vs {}'.format(
                    lane.get_previous_car_current_speed(car.on_position), lane.speed)

                # 1.1. 在当前道路上继续前行, 完成本次调度
                if car.on_position > car.current_speed:  # 车辆不足以穿过路口
                    # logger.info('2 {} 无前车, 车辆距离路口: {}, 当前车速为: {}, (道路限速: {}, 车辆最大速度: {}), 因此不足以通过路口'.format(car.car_id, car.on_position, car.current_speed, lane.speed, car.highest_speed))
                    # 无前车, 可以看作一直在跟车 (跟路口)
                    self._car_moves_on_the_same_way(car, lane, car.current_speed, is_following=True)
                    car.state = CAR_STOP  # 状态更新放在这里, 方便统一检查
                    continue
                
                elif car.on_position == car.current_speed:
                    self._car_moves_on_the_same_way(car, lane, car.on_position, is_following=True)
                    car.state = CAR_STOP  # 状态更新放在这里, 方便统一检查
                    continue

                # 1.2 经过路口的
                elif car.current_speed > car.on_position:

                    # 1.2.1 到达目的地
                    if car.start_cross_id == car.end_cross_id:
                        lane.positions[car.on_position] = None
                        # car.on_road = None
                        # car.on_lane = None
                        # car.on_position = None
                        car.state = CAR_END
                        self.running_cars.remove(car)
                        self.ended_cars.append(car)

                        self._update_road_weight(road)
                        continue

                    if not car.pass_intention:
                        self._make_plan_for_running_car(car)
                        # self._choose_a_road_to_turn(car)
                    car.state = CAR_RUNNING
                    continue

            # 2. 前车的调度已经完成, 放心地往前开就是
            elif previous_car.state == CAR_STOP:  
                previous_car_position = previous_car.on_position
                distance_from_previous_car = car.on_position - previous_car_position
                # logger.info('1. {} 前车已完成调度, 距离前车: {}, 当前车速为: {}'.format(car.car_id, distance_from_previous_car, car.current_speed))

                # 车辆以单位时间内可前进的距离前进, 不会到达跟车状态
                if distance_from_previous_car > car.current_speed:
                    self._car_moves_on_the_same_way(car, lane, car.current_speed, is_following=False)
                # 车辆以单位时间内可前进的距离前进, 达到跟车状态
                elif distance_from_previous_car == car.current_speed:
                    self._car_moves_on_the_same_way(car, lane, car.current_speed-1, is_following=True)
                # 当前道路上的可前进距离小于车辆单位时间能可行进的距离, 车辆可能穿过路口
                car.state = CAR_STOP
                continue

            elif previous_car.state == CAR_RUNNING:
                if not car.pass_intention:
                    self._make_plan_for_running_car(car)
                    # self._choose_a_road_to_turn(car)
                car.state = CAR_RUNNING
                continue
            else:
                raise RuntimeError('迷路了吧')

    def _car_pass_cross(self, car, road_to_turn):
        lane_to_turn = road_to_turn.allocate_lane()
        source_road = self.roads.get(car.on_road)
        source_lane = source_road.lanes[int(car.on_lane[car.on_lane.find('@')+1:])-1]

        # 可能无法获得车道, 无法通过路口
        if not lane_to_turn:
            # logger.info('未能分配到车道, 切换道路失败')
            self._car_moves_on_the_same_way(
                car,
                source_lane,
                car.on_position,
                is_following=True)
            return

        # 速度不够, 无法通过路口
        distance_on_new_lane = min(lane_to_turn.speed, car.highest_speed) - car.on_position
        if distance_on_new_lane <= 0:
            # logger.info('车辆在单位时间内无法完成道路切换')
            self._car_moves_on_the_same_way(
                car,
                source_lane,
                car.on_position,
                is_following=True)
            return

            # 车辆上路, 变更量检查
            # 1. 车要变
                # 1. 车速可能会变 (current_speed)
                # 2. 车的起始路口要变 (start_cross_id)
                # 3. 车绑定的道路/车道/位置信息都要变 (on_road, on_lane, on_position)
            # 2. 车道要变
                # 1. 车道的某个位置要被车辆填充, 原来的位置要释放 (positions)
                # 2. 车道的容量要变 (随车辆的进出而变, 是 postitions 的一个函数)
                # 3. 车道的速度要变 (随车辆的进出而变, 也是 positions 的一个函数, 更确切地说, 是末位车的车速)
            # 3. 车道所在道路要变
                # 1. 道路的容量要变 (是车道容量的函数)
                # 2. 道路状态可能会变 (车道容量的函数)
            # 4. 路网要变
                # 1. 最重要的, 对应道路的权重要变

        if lane_to_turn.get_current_capacity() > distance_on_new_lane:
            # 无法达到跟车状态, 车速限制在最大车速或道路限速
            # logger.info('第{t}个时间片调度\t车辆{car_id}即将进入的下一条道路容量为{capacity}, 车辆无阻挡情况下可行进{distance}, 无法达到跟车状态'.format(
                # t=self.current_time,
                # car_id=car.car_id,
                # capacity=lane_to_turn.get_current_capacity(),
                # distance=distance_on_new_lane))
            source_lane.positions[car.on_position] = None  # 从原车道上取出
            car.on_position = lane_to_turn.capacity-distance_on_new_lane
            car.current_speed = min(lane_to_turn.speed, car.highest_speed)
            lane_to_turn.positions[-distance_on_new_lane] = car

        else:
            # logger.info('第{t}个时间片调度\t车辆{car_id}即将进入的下一条道路容量为{capacity}, 车辆无阻挡情况下可行进{distance}, 达到跟车状态'.format(
            #     t=self.current_time,
            #     car_id=car.car_id,
            #     capacity=lane_to_turn.get_current_capacity(),
            #     distance=distance_on_new_lane))
            source_lane.positions[car.on_position] = None
            car.on_position = lane_to_turn.find_last_drivein_position()  # 必须放在车辆进入车道的位置之前, 否则会后移一位
            car.current_speed = min(lane_to_turn.get_last_car_current_speed(), car.highest_speed)
            lane_to_turn.positions[lane_to_turn.find_last_drivein_position()] = car

        car.start_cross_id = road_to_turn.end_cross_id
        car.passed_roads.append(road_to_turn.road_id)
        car.passed_crosses.append(road_to_turn.start_cross_id)
        car.on_road = road_to_turn.road_id
        car.on_lane = lane_to_turn.lane_id
        assert lane_to_turn.positions[car.on_position] is car
        car.state = CAR_STOP

        # 更新道路的权重, 可能新上路的车只能开到车道的最末位, 这时候车道相当于直接报废了, 此时无法再次获得车道信息
        self._update_road_weight(road_to_turn)

        # TODO: 这一操作严重影响了程序的运行速度
        # 更新原先道路的权重, 可能原先道路上只有本车
        # 在有后车的情况下, 路权已经在第一次调度中更新过了
        # self._update_road_weight(source_road)

    def _car_moves_on_the_same_way(self, car, lane, forward_distance, is_following):
        """车辆不会穿过路口"""

        # 在调度当前车辆之前, 获取前车的当前车速, 可能是当前车道的车速
        if lane.find_previous_car_position(car.on_position) is None:
            previous_car_current_speed = lane.speed
        else:
            previous_car_current_speed = lane.positions[lane.find_previous_car_position(car.on_position)].current_speed

        # 更新车道信息
        # NOTE: 不能先前进再释放, 可能导致车辆不前进被覆盖
        lane.positions[car.on_position] = None                  # 释放当前位置
        lane.positions[car.on_position-forward_distance] = car  # 车辆前进到目标位置

        # 更新车辆信息
        if is_following:
            # 达到跟车状态, 车辆的当前速度由前车与车辆的最大速度决定
            car.current_speed = min(previous_car_current_speed, car.highest_speed)
        car.on_position = car.on_position - forward_distance           # 变更当前所在位置
        car.state = CAR_STOP

        # 更新后车的当前车速
        if lane.find_next_car_position(car.on_position) != None:
            next_car = lane.positions[lane.find_next_car_position(car.on_position)]
            next_car.current_speed = min(car.current_speed, next_car.highest_speed)
    
    def _update_road_weight(self, road):
        lane = road.allocate_lane()
        if lane:
            self.roadnet[road.start_cross_id][road.end_cross_id]['weight'] = \
                road.length / lane.get_last_car_current_speed()
        else:
            self.roadnet[road.start_cross_id][road.end_cross_id]['weight'] = 1000

    def get_cross_states_where_car_is(self, car):
        cross = self.crosses.get(car.start_cross_id)
        road_states = []

        for road in cross.connected_roads.values():
            if not road:
                continue
            if road.road_id == car.on_road:
                continue
            road_states.append(road.get_current_state())
        
        if all([state == BLOCKED for state in road_states]):
            return BLOCKED
        elif DRIVEIN_ABLE in road_states:
            return DRIVEIN_ABLE
        else:
            return TO_BE_SCHEDULED

    def _choose_a_road_to_turn(self, car, num_path=10):
        """车库中的车辆上路, 为其选择一条道路"""
        cross = self.crosses.get(car.start_cross_id)
        neighbor_cross_ids = list(self.roadnet[cross.cross_id].keys())
        neighbor_cross_ids.remove(car.passed_crosses[-1])

        # 可选的道路, connected_rodas 记录的是进入路口的道路
        roads = [self.cross_pair_to_road.get((cross.cross_id, neighbor_cross_id))
                for neighbor_cross_id in neighbor_cross_ids]
        roads = [road for road in roads
                if car.on_road != road.road_id and road.get_current_state() != BLOCKED]

        assert roads is not None

        # 可选道路的尽头路口
        cross_ids = [road.end_cross_id for road in roads]

        path_and_time = []
        for road, cross_id in zip(roads, cross_ids):
            inner_path_and_time = []
            for i, path in enumerate(nx.shortest_simple_paths(self.roadnet, cross_id, car.end_cross_id)):
                if len(path_and_time) > num_path or i > 100:
                    break

                # 加上当前路口
                path_time = self.get_path_time(path)
                if path_time > 1000:
                    continue
                path_and_time.append(([cross.cross_id]+path, path_time+self.roadnet[cross.cross_id][path[0]]['weight']))

            path_and_time.extend(inner_path_and_time)


        paths, times = zip(*path_and_time)
        index = np.random.choice(range(len(paths)), p=np.array([1/t for t in times])/sum([1/t for t in times]))

        road_to_turn = self.cross_pair_to_road.get((paths[index][0], paths[index][1]))
        lane_to_turn = road_to_turn.allocate_lane()

        assert paths[index][0] == car.start_cross_id
        assert lane_to_turn is not None
        assert cross.road_pair2pass_way[(car.on_road[:car.on_road.find('#')], road_to_turn.road_id[:road_to_turn.road_id.find('#')])]

        car.pass_intention = cross.road_pair2pass_way[(car.on_road[:car.on_road.find('#')], road_to_turn.road_id[:road_to_turn.road_id.find('#')])]
        car.road_to_turn = road_to_turn.road_id

    def _schedule_cars_to_run(self):
        # step1, 计算当前路网的容量
        #        1. 汇总每条道路的容量
        #        2. 每条道路的容量由道路的车道容量而来
        #        3. 车道容量由车道上车辆的位置决定

        # 容量大, 方可允许车辆上路; 容量小于封锁容量, 禁止车辆上路
        current_roadnet_capacity = self.get_current_roadnet_capacity()
        remain_roadnet_capacity = current_roadnet_capacity - self.block_roadnet_capacity 
        if remain_roadnet_capacity <= 0:
            logger.info(
                '第{t}个时间片调度\t当前剩余路网容量为{capacity}, 小于封锁容量{block_roadnet_capacity}. 禁止发车!!!'.format(
                    t=self.current_time,
                    capacity=current_roadnet_capacity,
                    block_roadnet_capacity=self.block_roadnet_capacity))
            return

        # 检查是否有车辆准备上路
        current_cars_to_run = self._find_cars_to_run(self.num_cars_on_road)
        if len(current_cars_to_run) == 0:
            logger.info('第{t}个时间片调度\t当前没有计划出发的车辆'.format(t=self.current_time))
            return

        # step2, 根据当前路网的容量, 与车辆所在路口相连的道路的容量, 选择车辆上路
        logger.info('第{t}个时间片调度\t{n}辆车已经到达计划出发时间'.format(
            t=self.current_time,
            n=min(remain_roadnet_capacity, len(current_cars_to_run))))

        # 车辆持续上路, 直到达到封锁条件
        while self.get_current_roadnet_capacity() > self.block_roadnet_capacity or self.num_cars_on_road > len(self.running_cars):
            # 按车辆 id 升序发车
            for car in current_cars_to_run:
                if car.planned_departure_time > self.current_time:
                    continue

                # 车辆上路之后, 路网信息会发生改变, 因此, 在之前的规划基础上, 重新为当前车辆规划路线
                self._make_plan_for_car_to_run(car)

                # TODO: 车辆所在路口的局部容量
                road_to_run = self._choose_a_road_to_run(car)

                if not road_to_run or road_to_run.get_current_state() != DRIVEIN_ABLE:
                    logger.info('第{t}个时间片调度\t没有为编号为{car_id}的车辆找到合适的出发道路或道路阻塞, 暂缓出发'.format(
                        t=self.current_time, car_id=car.car_id))
                    # cars_cannot_start_off.append(car)
                    continue

                # 车辆上路之前, 分配车道
                lane = road_to_run.allocate_lane()
                assert lane is not None  # 道路不阻塞, 理论上就能分配到车道

                # 车辆上路, 变更量检查
                # 1. 车要变
                    # 1. 车速可能会变 (current_speed)
                    # 2. 车的起始路口要变 (start_cross_id)
                    # 3. 车绑定的道路/车道/位置信息都要变 (on_road, on_lane, on_position)
                # 2. 车道要变
                    # 1. 车道的某个位置要被车辆填充 (positions)
                    # 2. 车道的容量要变 (随车辆的进出而变, 是 postitions 的一个函数)
                    # 3. 车道的速度要变 (随车辆的进出而变, 也是 positions 的一个函数, 更确切地说, 是末位车的车速)
                # 3. 车道所在道路要变
                    # 1. 道路的容量要变 (是车道容量的函数)
                    # 2. 道路状态可能会变 (车道容量的函数)
                # 4. 路网要变
                    # 1. 最重要的, 对应道路的权重要变

                if lane.get_current_capacity() > car.current_speed:
                    # 无法达到跟车状态, 车速保持不变
                    lane.positions[lane.capacity-car.current_speed] = car
                    car.on_position = lane.capacity-car.current_speed
                else:
                    # 会达到跟车状态, 车辆行驶到前车之后, 速度发生变更
                    car.on_position = lane.find_last_drivein_position()  # 必须放在车辆进入车道的位置之前, 否则会后移一位
                    car.current_speed = min(lane.get_previous_car_current_speed(car.on_position), car.current_speed)
                    lane.positions[lane.find_last_drivein_position()] = car

                # logger.info('第{t}个时间片调度\t车辆上路后, 道路{road_id}的路况变为'.format(t=self.current_time, road_id=road_to_run.road_id))
                # for lan in road_to_run.lanes:
                #     logger.info('\t{lane_id}的剩余容量为: {capacity}, 路况为:{pos}'.format(
                #         lane_id=lan.lane_id, capacity=lan.get_current_capacity(), pos=lan.positions))

                car.start_cross_id = road_to_run.end_cross_id
                car.departure_time = self.current_time
                car.passed_roads.append(road_to_run.road_id)
                car.passed_crosses.append(road_to_run.start_cross_id)
                car.on_road = road_to_run.road_id
                car.on_lane = lane.lane_id
                assert lane.positions[car.on_position] is car

                # 上路的车辆加入 running_cars
                self.cars_to_run.remove(car)
                self.running_cars.append(car)
                assert car in self.running_cars

                # 更新道路的权重, 可能新上路的车只能开到车道的最末位, 这时候车道相当于直接报废了, 此时无法再次获得车道信息
                relane = road_to_run.allocate_lane()
                if relane:
                    self.roadnet[road_to_run.start_cross_id][road_to_run.end_cross_id]['weight'] = \
                        road_to_run.length / relane.get_last_car_current_speed()
                else:
                    self.roadnet[road_to_run.start_cross_id][road_to_run.end_cross_id]['weight'] = 1000
            else:
                break
        logger.info('第{t}个时间片调度\t上路车辆调度完成'.format(t=self.current_time))

    def get_current_roadnet_capacity(self):
        return sum([road.get_current_capacity() for road in self.roads.values()])

    def _arrange_cars_to_run(self):
        """根据当前路况, 重新编排发车顺序"""
        # NOTE: 当数据量很大时, 这是个十分耗时的活计

        # TODO: 更复杂的方法是对于后面的道路, 估计到达时间, 然后计算那个时候的条件
        # TODO: weight=func() 动态地计算每条路上的 weight, 是个精细活.
        for car in self.cars_to_run:
            self._make_plan_for_car_to_run(car)

        # self.cars_to_run = sorted(self.cars_to_run, key=lambda car: int(car.car_id))
        self.cars_to_run = sorted(self.cars_to_run, key=lambda car: car.ideal_arrival_time or int(car.car_id))

    def _make_plan_for_running_car(self, car):
        for path in nx.shortest_simple_paths(self.roadnet,
                                             car.start_cross_id,
                                             car.end_cross_id,
                                             weight='weight'):
            if len(path) == 1:
                return
            if path[1] == car.passed_crosses[-1]:
                continue
            # 找到第一条不是掉头的路
            car.ideal_path = path
            break
        car.ideal_time = self.get_path_time(car.ideal_path)

        road_to_turn = self.cross_pair_to_road.get((car.ideal_path[0], car.ideal_path[1]))
        cross = self.crosses.get(car.start_cross_id)
        assert cross.road_pair2pass_way[(car.on_road[:car.on_road.find('#')], road_to_turn.road_id[:road_to_turn.road_id.find('#')])]
        car.pass_intention = cross.road_pair2pass_way[(car.on_road[:car.on_road.find('#')], road_to_turn.road_id[:road_to_turn.road_id.find('#')])]
        car.road_to_turn = road_to_turn.road_id
    
    def _make_plan_for_car_to_run(self, car):
        car.ideal_path = nx.dijkstra_path(self.roadnet,
                                          car.start_cross_id,
                                          car.end_cross_id,
                                          weight='weight')
        car.ideal_time = self.get_path_time(car.ideal_path)
        car.ideal_arrival_time = max(car.planned_departure_time, self.current_time) + car.ideal_time


    def get_path_time(self, path):
        return sum([
            self.roadnet[s][e]['weight']
            for s, e in zip(path[:-1], path[1:])
        ])
    
    def _find_cars_to_run(self, k=None):
        """当前时间已经到了或者过了车辆的计划出发时间, 车辆可出发"""
        current_cars_to_run = [car for car in self.cars_to_run if car.planned_departure_time <= self.current_time]
        if k:
            return sorted(current_cars_to_run, key=lambda c: int(c.car_id))[:k]
        else:
            return current_cars_to_run

    def _choose_a_road_to_run(self, car, num_path=10, prob4ideal_path=0.5):
        """车库中的车辆上路, 为其选择一条道路"""
        ideal_path = car.ideal_path
        shortest_simple_paths = nx.shortest_simple_paths(self.roadnet, car.start_cross_id, car.end_cross_id)
        path_and_time = []  # 除最优路径以外的其他可用路径的耗时
        for i, path in enumerate(shortest_simple_paths):
            if path == ideal_path:
                continue
            # 限时要求, 仅探索有限的路线
            if len(path_and_time) > num_path or i > 100:
                break

            # step1: 根据路口信息, 获得道路信息
            # TODO: 不止考虑当前与当前路口相连的道路的封锁情况
            if self.cross_pair_to_road.get((path[0], path[1])).get_current_state() == BLOCKED:
                continue

            path_time = self.get_path_time(path)
            # if path_time > 1000:
            #     continue
            path_and_time.append((path, path_time))

        # 遍历所有可能路径未发现可走路线的, 车辆不上路
        if not path_and_time \
           and self.cross_pair_to_road.get((ideal_path[0], ideal_path[1])).get_current_state() == BLOCKED:
            return None

        # 只有最优路径可走的情况
        if not path_and_time:
            return self.cross_pair_to_road.get((ideal_path[0], ideal_path[1]))

        paths, times = zip(*path_and_time)
        if self.cross_pair_to_road.get((ideal_path[0], ideal_path[1])).get_current_state() == BLOCKED:
            # 最优路径不可走
            path = paths[np.random.choice(range(len(paths)), p=np.array([1/t for t in times])/sum([1/t for t in times]))]
            return self.cross_pair_to_road.get((path[0], path[1]))
        else:
            # 最优路径可走的情况下, 以预设概率走最优路径
            if random.random() < prob4ideal_path:
                return self.cross_pair_to_road.get((ideal_path[0], ideal_path[1]))
            else:
                # path = np.random.choice(paths, p=np.array([1/t for t in times])/sum([1/t for t in times]))
                path = paths[np.random.choice(range(len(paths)), p=np.array([1/t for t in times])/sum([1/t for t in times]))]
                return self.cross_pair_to_road.get((path[0], path[1]))