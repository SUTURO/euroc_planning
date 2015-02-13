import rospy
import time
from math import pi
from suturo_planning_interface import mathemagie
from suturo_planning_interface import utils
from geometry_msgs.msg import Point
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceResponse
from suturo_planning_interface.mathemagie import euclidean_distance, get_angle, subtract_point
from suturo_planning_manipulation.calc_grasp_position import make_scan_pose
from suturo_planning_visualization.visualization import visualize_poses


class ScanObstacles(object):
    def __init__(self, service_name="suturo/state/scan_obstacles"):
        self.classified_regions = []
        self.next_cluster = 0
        rospy.Service(service_name, TaskDataService, self.__handle_call)

    def __handle_call(self, req):
        response = TaskDataServiceResponse()
        taskdata = req.taskdata
        response.result = self.__scan_obstacles(taskdata)
        response.taskdata = taskdata
        return response

    def __scan_obstacles(self, taskdata):
        rospy.loginfo('Executing state ScanObstacles')
        taskdata.sec_try_done = False
        if taskdata.sec_try:
            rospy.loginfo("scan_obstacles: sec_try")
            current_region = self.classified_regions[self.next_cluster-1][0]
            taskdata.sec_try_done = True
        else:
            #get regions
            if len(self.classified_regions) == 0:
                rospy.loginfo("scan_obstacles: classified_regions == 0 first try")
                obstacle_cluster = utils.map.get_obstacle_regions()
                rospy.logdebug(str(len(self.classified_regions)) + " regions found.")
                print '#####################################'
                self.classified_regions = utils.map.undercover_classifier(obstacle_cluster, taskdata.yaml.objects)
                print self.classified_regions
                print '#####################################'
                base = utils.manipulation.get_base_origin().point
                self.classified_regions.sort(key=lambda x: euclidean_distance(Point(*(utils.map.index_to_coordinates(*x[0].get_avg()))+(0.0,)), base))
                # x[0].get_number_of_cells())

            if self.next_cluster >= len(self.classified_regions):
                rospy.loginfo("scan_obstacles: searched all cluster")
                rospy.loginfo("searched all cluster")
                return 'noRegioqnLeft'

            current_region = self.classified_regions[self.next_cluster][0]
            rospy.logdebug("current region: " + str(self.next_cluster) + "\n" + str(current_region) +
                           "\nclassified as " + str(self.classified_regions[self.next_cluster][1]))
            self.next_cluster += 1

        region_centroid = Point(*(utils.map.index_to_coordinates(*current_region.get_avg()))+(-0.065,))

        dist_to_region = mathemagie.euclidean_distance(Point(0, 0, 0), region_centroid)

        #if userdata.yaml.task_type == Task.TASK_5:
        #    fixture_position = mathemagie.add_point(userdata.yaml.puzzle_fixture.position, Point(0.115, 0.165, 0))
        #    dist_to_fixture = mathemagie.euclidean_distance(fixture_position, region_centroid)
        #    if dist_to_fixture < 0.35:
        #       rospy.logdebug("Region classified as puzzle fixture, skipping")
        #       return 'mapScanned'

        # If the arm cannot move ignore distant regions
        # TODO find the best max distance
        if not taskdata.enable_movement:
            rospy.logdebug('Distance of the current region to the arm: %s' % str(dist_to_region))
            if dist_to_region > 1.1:
                rospy.logwarn('Current region is out of reach. Ignoring it.')
                return 'mapScanned'

        rospy.loginfo("scan_obstacles: the arm movement to an obstacle")

        angle = 1.2
        distance = 0.6 + current_region.get_number_of_cells()*0.008

        poses = make_scan_pose(region_centroid, distance, angle, n=16)

        if not taskdata.enable_movement:
            poses = utils.manipulation.filter_close_poses(poses)

        poses = utils.map.filter_invalid_scan_poses2(region_centroid.x, region_centroid.y, poses)

        if taskdata.sec_try:
            rospy.loginfo("scan_obstacles: the arm movement to an obstacle; sec_try")
            current_pose = utils.manipulation.get_eef_position().pose.position
            current_pose.z = 0
            region_to_eef = subtract_point(region_centroid, current_pose)

            poses.sort(key=lambda pose: abs(get_angle(region_to_eef, subtract_point(Point(pose.pose.position.x, pose.pose.position.y, 0), region_centroid)) - pi/2))

        visualize_poses(poses)

        if taskdata.enable_movement:
            # move = utils.manipulation.move_arm_and_base_to
            plan = utils.manipulation.plan_arm_and_base_to
        else:
            # move = utils.manipulation.move_to
            plan = utils.manipulation.plan_arm_to

        utils.manipulation.blow_up_objects(do_not_blow_up_list=("map"))
        for pose in poses:
            # utils.manipulation.set_planning_time_arm(2)
            if utils.manipulation.move_with_plan_to(plan(pose)):
                # utils.manipulation.set_planning_time_arm(5)
                taskdata.focused_point = region_centroid

                rospy.logdebug('Wait for clock')
                time.sleep(0.5)
                rospy.sleep(2.5)
                return 'newImage'
        utils.manipulation.blow_down_objects()
        return 'mapScanned'


if __name__ == "__main__":
    import rospy
    rospy.init_node("scan_obstacles")
    s = ScanObstacles()
    while True:
        rospy.spin()