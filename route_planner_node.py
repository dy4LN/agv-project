#!/usr/bin/env python3

import rospy
import osmnx as ox
import json
import math
from shapely.geometry import shape

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint


# =====================================================
# DESTINATION LOOKUP TABLE
# tag_id → (lat, lon, name)
# =====================================================

DESTINATIONS = {
    0: {"name": "Skye Hall",          "lat": 33.9746, "lon": -117.3265},
    1: {"name": "Bourns Hall",        "lat": 33.9724, "lon": -117.3283},
    2: {"name": "Winston Chung Hall", "lat": 33.9738, "lon": -117.3271},
    3: {"name": "HUB",                "lat": 33.9731, "lon": -117.3260},
    4: {"name": "Rivera Library",     "lat": 33.9750, "lon": -117.3280},
}

BOUNDARY_GEOJSON = "/home/dylan42603/catkin_ws/src/agv_navigation/config/ucr_boundary.geojson"

# Max spacing (meters) between interpolated waypoints
INTERPOLATION_SPACING = 2.0


# =====================================================
# ROUTE PLANNER NODE
# =====================================================

class AGVRoutePlanner:

    def __init__(self):

        rospy.init_node("agv_route_planner")

        # =============================
        # STATE
        # =============================
        self.graph = None
        self.current_lat = None
        self.current_lon = None
        self.from_ll_srv = None

        # =============================
        # PUBLISHERS
        # =============================
        self.path_pub = rospy.Publisher("/waypoints", Path, queue_size=10, latch=True)
        self.route_ready_pub = rospy.Publisher("/route_ready", Bool, queue_size=10)

        # =============================
        # LOAD OSM GRAPH
        # =============================
        rospy.loginfo("[PLANNER] Loading OSM graph...")
        try:
            self._load_graph()
            rospy.loginfo(
                f"[PLANNER] Graph loaded: "
                f"{len(self.graph.nodes)} nodes, {len(self.graph.edges)} edges"
            )
        except Exception as e:
            rospy.logerr(f"[PLANNER] Failed to load graph: {e}")
            return

        # =============================
        # WAIT FOR /fromLL SERVICE
        # =============================
        rospy.loginfo("[PLANNER] Waiting for /fromLL...")
        try:
            rospy.wait_for_service("/fromLL", timeout=15.0)
            self.from_ll_srv = rospy.ServiceProxy("/fromLL", FromLL)
            rospy.loginfo("[PLANNER] /fromLL ready.")
        except rospy.ROSException:
            rospy.logerr("[PLANNER] /fromLL not available (navsat_transform_node?)")
            return

        # =============================
        # SUBSCRIBERS
        # =============================
        rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/desired_tag_id", Int32, self.tag_callback)

        rospy.loginfo("[PLANNER] Ready. Waiting for destination...")
        rospy.spin()

    # =====================================================
    # GRAPH LOADING
    # =====================================================

    def _load_graph(self):
        with open(BOUNDARY_GEOJSON) as f:
            geojson_data = json.load(f)

        polygon = shape(geojson_data["features"][0]["geometry"])
        self.graph = ox.graph_from_polygon(polygon, network_type="all")

    # =====================================================
    # CALLBACKS
    # =====================================================

    def gps_callback(self, msg):
        """Continuously update current GPS position."""
        if msg.status.status >= 0:
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude

    def tag_callback(self, msg):

        tag_id = msg.data

        if tag_id < 0:
            rospy.loginfo("[PLANNER] Ignoring negative tag_id.")
            return

        if tag_id not in DESTINATIONS:
            rospy.logwarn(f"[PLANNER] Invalid tag_id {tag_id}")
            return

        if self.current_lat is None or self.current_lon is None:
            rospy.logwarn("[PLANNER] No GPS fix yet.")
            return

        dest = DESTINATIONS[tag_id]

        rospy.loginfo(
            f"[PLANNER] Goal: {dest['name']} "
            f"({dest['lat']:.6f}, {dest['lon']:.6f})"
        )

        self._plan_and_publish(
            self.current_lat,
            self.current_lon,
            dest["lat"],
            dest["lon"],
        )

    # =====================================================
    # ROUTE PLANNING
    # =====================================================

    def _plan_and_publish(self, start_lat, start_lon, goal_lat, goal_lon):

        G = self.graph

        rospy.loginfo(
            f"[PLANNER] Planning route "
            f"({start_lat:.6f},{start_lon:.6f}) → "
            f"({goal_lat:.6f},{goal_lon:.6f})"
        )

        orig = ox.distance.nearest_nodes(G, start_lon, start_lat)
        dest = ox.distance.nearest_nodes(G, goal_lon, goal_lat)

        route = ox.shortest_path(G, orig, dest, weight="length")

        if route is None:
            rospy.logerr("[PLANNER] No route found.")
            return

        rospy.loginfo(f"[PLANNER] Raw route: {len(route)} nodes")

        route_latlon = [(G.nodes[n]["y"], G.nodes[n]["x"]) for n in route]

        route_latlon = self._interpolate(route_latlon, INTERPOLATION_SPACING)

        rospy.loginfo(f"[PLANNER] Interpolated: {len(route_latlon)} waypoints")

        local_coords = self._convert_to_local(route_latlon)

        if local_coords is None:
            rospy.logerr("[PLANNER] Conversion failed.")
            return

        path_msg = self._build_path_msg(local_coords)

        self.path_pub.publish(path_msg)
        self.route_ready_pub.publish(Bool(True))

        rospy.loginfo(f"[PLANNER] Published {len(local_coords)} waypoints.")

    # =====================================================
    # LAT/LON → LOCAL MAP (via robot_localization)
    # =====================================================

    def _convert_to_local(self, latlon_list):

        local = []

        for lat, lon in latlon_list:
            try:
                geo_pt = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
                resp = self.from_ll_srv(geo_pt)
                local.append((resp.map_point.x, resp.map_point.y))

            except rospy.ServiceException as e:
                rospy.logerr(f"[PLANNER] fromLL failed: {e}")
                return None

        return local

    # =====================================================
    # BUILD PATH MESSAGE
    # =====================================================

    def _build_path_msg(self, coords):

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for x, y in coords:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    # =====================================================
    # INTERPOLATION
    # =====================================================

    def _interpolate(self, coords, max_spacing):

        if len(coords) < 2:
            return coords

        result = [coords[0]]

        for i in range(1, len(coords)):
            lat1, lon1 = coords[i - 1]
            lat2, lon2 = coords[i]

            d = self._haversine(lat1, lon1, lat2, lon2)

            if d > max_spacing:
                steps = int(math.ceil(d / max_spacing))
                for s in range(1, steps):
                    t = s / steps
                    result.append((
                        lat1 + t * (lat2 - lat1),
                        lon1 + t * (lon2 - lon1),
                    ))

            result.append(coords[i])

        return result

    # =====================================================
    # HAVERSINE DISTANCE
    # =====================================================

    def _haversine(self, lat1, lon1, lat2, lon2):

        R = 6371000.0

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)

        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = (
            math.sin(dphi / 2) ** 2
            + math.cos(phi1) * math.cos(phi2)
            * math.sin(dlambda / 2) ** 2
        )

        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# =====================================================
# ENTRY POINT
# =====================================================

if __name__ == "__main__":
    try:
        AGVRoutePlanner()
    except rospy.ROSInterruptException:
        pass
