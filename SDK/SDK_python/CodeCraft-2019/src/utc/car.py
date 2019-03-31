CAR_TO_RUN = -1
CAR_RUNNING = 1
CAR_STOP = 2
CAR_END = 3



class Car(object):
    def __init__(self, car_id, start_cross_id, end_cross_id,
                 highest_speed, planned_departure_time):
        super(Car, self).__init__()

        self.car_id = str(car_id)
        self.start_cross_id = str(start_cross_id)
        self.end_cross_id = str(end_cross_id)
        self.highest_speed = int(highest_speed)
        self.planned_departure_time = int(planned_departure_time)

        self.state = None
        self.current_speed = self.highest_speed

        self.ideal_time = None
        self.ideal_arrival_time = None
        self.ideal_path = None

        self.departure_time = None  # departure time
        self.passed_roads = []
        self.passed_crosses = []
        
        self.on_road = None  # road id
        self.on_lane = None  # lane id
        self.on_position = None  # road position on lane

        self.pass_intention = None
        self.road_to_turn = None

    def __str__(self):
        return self.car_id

    def __repr__(self):
        return self.car_id