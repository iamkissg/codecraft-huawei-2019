from collections import OrderedDict

class Cross(object):
    def __init__(self, cross_id, connected_roads):
        assert len(connected_roads) == 4
        self.cross_id = str(cross_id)

        self.connected_roads = connected_roads
        self.generate_road_pair_and_pass_way_mapping(connected_roads)
        
    def generate_road_pair_and_pass_way_mapping(self, roads):
        pass_way2road_pair = {'turn_left': [], 'go_straight': [], 'turn_right': []}
        road_pair2pass_way = {}
        for i in range(len(roads)):
            if roads[i] == '-1':
                continue
            if roads[(i+1)%4] != '-1':
                pass_way2road_pair['turn_left'].append((roads[i], roads[(i+1)%4]))
                road_pair2pass_way[(roads[i], roads[(i+1)%4])] = 'turn_left'
            if roads[(i+2)%4] != '-1':
                pass_way2road_pair['go_straight'].append((roads[i], roads[(i+2)%4]))
                road_pair2pass_way[(roads[i], roads[(i+2)%4])] = 'go_straight'
            if roads[(i+3)%4] != '-1':
                pass_way2road_pair['turn_right'].append((roads[i], roads[(i+3)%4]))
                road_pair2pass_way[(roads[i], roads[(i+3)%4])] = 'turn_right'
        self.pass_way2road_pair = pass_way2road_pair
        self.road_pair2pass_way = road_pair2pass_way

    def connect_with_roads(self, roads_dict):
        connected_roads = OrderedDict()
        for road_id in sorted(self.connected_roads,  key=lambda t: int(t)):
            if road_id == '-1':
                if '-1' not in connected_roads:
                    connected_roads['-1'] = None
                else:
                    connected_roads['-2'] = None
                continue
            for i in range(1, 3):
                rid = road_id+'#{}'.format(i)
                road = roads_dict.get(rid)
                if not road:
                    continue
                if road.end_cross_id == self.cross_id:
                    connected_roads[road_id] = roads_dict.get(rid)
                    break
            else:
                if '-1' not in connected_roads:
                    connected_roads['-1'] = None
                else:
                    connected_roads['-2'] = None
                
        self.connected_roads = connected_roads
