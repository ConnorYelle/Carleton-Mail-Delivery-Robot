from tools.nav_parser import loadMap

beacon_ids = {
    "e2_77_fc_f9_04_93": "UC",
    "ea_2f_93_a6_98_20": "ResComms",
    "fc_e2_2e_62_9b_3d": "CTTC",
    "e4_87_91_3d_1e_d7": "Mackenzie/Minto",
    "ee_16_86_9a_c2_a8": "Frontenac",
    "d0_6a_d2_02_42_eb": "Nicol",
    "df_2b_70_a8_21_90": "Canal",
    "fb_ef_5c_de_ef_e4": "Robertson",
}

class Map:

    '''
    Defines the hashmap that the robot uses for its routes.
    Remember: the map has a hashmap within a hashmap as values. Therefore, you would retrieve a value from within the map as follows:

    routing_map["Loeb1"]["Southam"] -> returns U-TURN
    '''

    def __init__(self):
        '''
        Constructor for the map. Essentially initializes the map into a hashmap.
        '''
        self.routing_map = loadMap()

    def getMap(self):
        '''
        Typical getter for the whole routing map.
        '''
        return self.routing_map

    def getDirection(self, source, destination):
        '''
        General method to return the value from a given key in a key-value pair.

        @param source: the source of the trip.
        @param destination: the destination of the trip.
        '''
        mac_address = source[:-1]
        source_name = beacon_ids.get(mac_address, None)
        check_id = source_name + source[-1]
        if source_name is None or check_id not in self.routing_map:
            return 'GO'
        return self.routing_map[source_name + source[-1]][destination]

    def exists(self, location):
        '''
        Checks to see if the location exists in the map.

        @param location: the building to verify.
        '''
        return location in self.routing_map
