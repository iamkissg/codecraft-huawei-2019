import os
import logging
import sys
from time import time

import matplotlib.pyplot as plt

import networkx as nx
from utc.car import Car
from utc.cross import Cross
from utc.road import Road
from utc.scheduler import Scheduler
from utc.util import strip_parenthesis, read_file_and_yield_info


logging.basicConfig(level=logging.DEBUG,
                    filename=os.path.join(os.path.abspath(__file__), '../../../logs/CodeCraft-2019.log'),
                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S',
                    filemode='a')
logger = logging.getLogger()
stdout_handler = logging.StreamHandler(sys.stdout)
logger.addHandler(stdout_handler)


def read_cars(car_path):
    cars_info = read_file_and_yield_info(car_path)
    cars = [
        Car(car_id=car[0], start_cross_id=car[1], end_cross_id=car[2],
            highest_speed=car[3], planned_departure_time=car[4])
        for car in cars_info
    ]
    return cars

def read_crosses(cross_path):
    crosses_info = read_file_and_yield_info(cross_path)
    crosses = [
        Cross(cross_id=cross[0], connected_roads=cross[1:])
        for cross in crosses_info
    ]
    return sorted(crosses, key=lambda cross: int(cross.cross_id))

def read_roads(road_path):
    roads_info = read_file_and_yield_info(road_path)
    roads = []
    for road in roads_info:
        if road[6] == '1':
            roads.extend([
                Road(road_id=road[0]+'#1', length=road[1], highest_speed=road[2], num_lane=road[3], start_cross_id=road[4], end_cross_id=road[5]),
                Road(road_id=road[0]+'#2', length=road[1], highest_speed=road[2], num_lane=road[3], start_cross_id=road[5], end_cross_id=road[4])
            ])
        else:
            roads.append(Road(road_id=road[0]+'#1', length=road[1], highest_speed=road[2], num_lane=road[3], start_cross_id=road[4], end_cross_id=road[5]))
    return sorted(roads,
                  key=lambda road: int(road.road_id[:road.road_id.find('#')])
                                   or int(road.road_id[road.road_id.find('#')+1:]))


def write_answer(answer_path, cars):
    with open(answer_path, 'w') as fout:
        for car in cars:
            answer = [car.car_id] + [str(car.departure_time)] + [road_id[:road_id.find('#')] for road_id in car.passed_roads]
            answer = '(' + ', '.join(answer) + ')\n'
            fout.write(answer)


def main():
    if len(sys.argv) != 5:
        logger.error('please input args: car_path, road_path, cross_path, answerPath')
        exit(1)

    car_path = sys.argv[1]
    road_path = sys.argv[2]
    cross_path = sys.argv[3]
    answer_path = sys.argv[4]

    logger.info("car_path is %s" % (car_path))
    logger.info("road_path is %s" % (road_path))
    logger.info("cross_path is %s" % (cross_path))
    logger.info("answer_path is %s" % (answer_path))

    program_start_time = time()
    cars = read_cars(car_path)
    # for car in cars:
    #     logger.info(car.__dict__)
    crosses = read_crosses(cross_path)
    # for cross in crosses:
    #     logger.info(cross.__dict__)
    roads = read_roads(road_path)
    # for road in roads:
    #     logger.info(road.__dict__)
    #     for lane in road.lanes:
    #         logger.info(lane.__dict__)
    # logger.info(cross_indexer)
    # logger.info(road_indexer)
    scheduler = Scheduler(crosses, roads, cars, capacity_threshold=0.5, num_cars_on_road=128)
    nx.draw(scheduler.roadnet)

    while scheduler.cars_to_run or scheduler.running_cars:
        scheduler.schedule()
    # for car in scheduler.cars_to_start:
    #     logger.info(car.__dict__)

    # print(scheduler.roadnet.nodes)
    # print(scheduler.roadnet.edges)

    # roadnet = RoadNet(crosses=crosses, roads=roads)
    # print('Vertexes', roadnet._vertexes)
    # print(len(roadnet._vertexes))
    # print('Edges', roadnet._edges)

    plt.savefig('directed_graph.png')
    program_running_time = time() - program_start_time
    logger.info('Total time: {}'.format(program_running_time))
    write_answer(answer_path, cars)



if __name__ == "__main__":
    main()