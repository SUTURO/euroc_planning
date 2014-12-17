class ScanObstacles():

    def __init__(self):
        self._classified_regions = []
        self._next_cluster = 0

    def scan_obstacles(self):
        repeat = True

        while(repeat):
            rospy.loginfo('Scanning for obstacles')
            userdata.sec_try_done = False
            if userdata.sec_try:
                current_region = self._classified_regions[self._next_cluster-1][0]
                userdata.sec_try_done = True
            else:
                #get regions
                if len(self._classified_regions) == 0:
                    self._get_regions()

                #TODO Wie dies hier umsetzen ???
                if self.next_cluster >= len(self.classified_regions):
                    rospy.loginfo("searched all cluster")
                    return 'noRegionLeft'

                current_region = self.classified_regions[self.next_cluster][0]
                rospy.logdebug("current region: " + str(self.next_cluster) + "\n" + str(current_region) +
                               "\nclassified as " + str(self.classified_regions[self.next_cluster][1]))
                self.next_cluster += 1

            region_centroid = Point(*(utils.map.index_to_coordinates(*current_region.get_avg()))+(-0.065,))

            dist_to_region = mathemagie.euclidean_distance(Point(0, 0, 0), region_centroid)

            # TODO find the best max distance
            if not userdata.enable_movement:
                rospy.logdebug('Distance of the current region to the arm: %s' % str(dist_to_region))
                if dist_to_region > 1.1:
                    rospy.logwarn('Current region is out of reach. Ignoring it.')
                else
                    repeat = False

        repeat = True
        while(repeat):
            angle = 1.2
            distance = 0.6 + current_region.get_number_of_cells()*0.008

            poses = make_scan_pose(region_centroid, distance, angle, n=16)

            if not userdata.enable_movement:
                poses = utils.manipulation.filter_close_poses(poses)

            poses = utils.map.filter_invalid_scan_poses2(region_centroid.x, region_centroid.y, poses)

            if userdata.sec_try:
                current_pose = utils.manipulation.get_eef_position().pose.position
                current_pose.z = 0
                region_to_eef = subtract_point(region_centroid, current_pose)

                poses.sort(key=lambda pose: abs(get_angle(region_to_eef, subtract_point(Point(pose.pose.position.x, pose.pose.position.y, 0), region_centroid)) - pi/2))

            visualize_poses(poses)

            if userdata.enable_movement:
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
                    userdata.focused_point = region_centroid

                    rospy.logdebug('Wait for clock')
                    time.sleep(0.5)
                    rospy.sleep(2.5)
                    repeat = False
            if repeat:
                utils.manipulation.blow_down_objects()

    def _get_regions(self):
        obstacle_cluster = utils.map.get_obstacle_regions()
        rospy.logdebug(str(len(self._classified_regions)) + " regions found.")
        print '#####################################'
        self._classified_regions = utils.map.undercover_classifier(obstacle_cluster, userdata.yaml.objects)
        print self.classified_regions
        print '#####################################'
        base = utils.manipulation.get_base_origin().point
        self._classified_regions.sort(key=lambda x: euclidean_distance(Point(*(utils.map.index_to_coordinates(*x[0].get_avg()))+(0.0,)), base))


