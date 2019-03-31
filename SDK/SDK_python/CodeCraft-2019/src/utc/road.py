import logging
from math import floor

from utc.car import CAR_STOP, CAR_RUNNING, CAR_TO_RUN

BLOCKED = 0
TO_BE_SCHEDULED = 1
DRIVEIN_ABLE = 2

logger = logging.getLogger()


class Lane(object):
    def __init__(self, lane_id, length, speed):
        super(Lane, self).__init__()
        self.lane_id = str(lane_id)
        self.capacity = int(length)  # remain capacity
        self.speed = int(speed)

        self.positions = [None for _ in range(self.capacity)]

    def find_last_drivein_position(self):
        for i, state in enumerate(self.positions[::-1]):
            if state is None:
                continue
            else:
                return self.capacity - i
        else:
            return 0

    def find_previous_car_position(self, pos):
        if pos == 0:
            return None
        for i, state in enumerate(self.positions[:pos][::-1], 1):
            if state is None:
                continue
            else:
                return pos - i
        else:
            return None

    def find_next_car_position(self, pos):
        if pos == -1 or pos == self.capacity - 1:
            return None
        for i, state in enumerate(self.positions[pos+1:], 0):
            if state is None:
                continue
            else:
                return pos + 1 + i
        else:
            return None

    def get_current_capacity(self, last_drivein_position=None):
        if last_drivein_position:
            return self.capacity - last_drivein_position
        else:
            return self.capacity - self.find_last_drivein_position()
    
    def get_last_car_current_speed(self):
        last_car_pos = self.find_last_drivein_position() - 1

        if last_car_pos == -1:
            return self.speed  # no car in the road, return road speed
        else:
            return self.positions[last_car_pos].current_speed

    def get_previous_car_current_speed(self, pos):
        pre_car_pos = self.find_previous_car_position(pos)

        if pre_car_pos == None:
            return self.speed  # no car in the road, return road speed
        else:
            return self.positions[pre_car_pos].current_speed


class Road(object):
    def __init__(
        self, road_id, length, highest_speed, num_lane,
        start_cross_id, end_cross_id, capacity_threshold=0.0):
        super(Road, self).__init__()

        self.road_id = str(road_id)
        self.length = int(length)
        self.num_lane = int(num_lane)
        self.highest_speed = int(highest_speed)
        self.start_cross_id = str(start_cross_id)
        self.end_cross_id = str(end_cross_id)

        self.max_capacity = self.length * self.num_lane
        self.block_capacity = floor(self.max_capacity * capacity_threshold)


        self.init_lane()


    def init_lane(self):
        self.lanes = [
            Lane(lane_id=self.road_id+'@'+str(n), length=self.length, speed=self.highest_speed)
            for n in range(1, self.num_lane+1)
        ]

    def get_current_capacity(self):
        return sum([lane.get_current_capacity() for lane in self.lanes])

    def allocate_lane(self):
        for lane in self.lanes:
            if lane.get_current_capacity() != 0:
                return lane
        else:
            None

    def get_current_state(self):
        if self.block_capacity >= self.get_current_capacity():
            lane_blocked = []
            for lane in self.lanes:
                last_car_on_lane = lane.positions[lane.find_last_drivein_position()-1]
                lane_blocked.append(last_car_on_lane.state == CAR_STOP)
            if all(lane_blocked):
                return BLOCKED
            else:
                return TO_BE_SCHEDULED
        else:
            return DRIVEIN_ABLE

    def __repr__(self):
        return self.road_id