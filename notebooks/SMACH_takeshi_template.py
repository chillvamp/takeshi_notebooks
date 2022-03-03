#!/usr/bin/env python
from std_srvs.srv import Empty, Trigger, TriggerRequest
import smach
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point , Quaternion
from actionlib_msgs.msg import GoalStatus
import moveit_commander
import moveit_msgs.msg
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from utils_notebooks import *
from utils_takeshi import *
########## Functions for takeshi states ##########
class Proto_state(smach.State):###example of a state definition.
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : PROTO_STATE')

        if self.tries==3:
            self.tries=0 
            return'tries'
        if succ:
            return 'succ'
        else:
            return 'failed'
        global trans_hand
        self.tries+=1
        if self.tries==3:
            self.tries=0 
            return'tries'

####################################################################Functions in notebook ##################################################################

#To Do: 
#To Do: pack them in an utils file
def pose_2_np(wp_p):
    
    #Takes a pose message and returns array
   
    return np.asarray((wp_p.pose.position.x,wp_p.pose.position.y,wp_p.pose.position.z)) , np.asarray((wp_p.pose.orientation.w,wp_p.pose.orientation.x,wp_p.pose.orientation.y, wp_p.pose.orientation.z)) 
def np_2_pose(position,orientation):
    #Takes a pose np array and returns pose message
    wb_p= geometry_msgs.msg.PoseStamped()
    
    wb_p.pose.position.x= position[0]
    wb_p.pose.position.y= position[1]
    wb_p.pose.position.z= position[2]
    wb_p.pose.orientation.w= orientation[0]
    wb_p.pose.orientation.x= orientation[1]
    wb_p.pose.orientation.y= orientation[2]
    wb_p.pose.orientation.z= orientation[3]
    return wb_p

def open_gripper():
    gcv=gripper.get_current_joint_values()
    gcv[2]= 0.8
    gripper.set_joint_value_target(gcv)
    gripper.go(gcv)
def close_gripper():
    gcv=gripper.get_current_joint_values()
    gcv[2]= 0.0
    gripper.set_joint_value_target(gcv)
    gripper.go(gcv)

def correct_points(low=.27,high=1000):

    #Corrects point clouds "perspective" i.e. Reference frame head is changed to reference frame map
    data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
    np_data=ros_numpy.numpify(data)
    trans,rot=listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) 
    
    eu=np.asarray(tf.transformations.euler_from_quaternion(rot))
    t=TransformStamped()
    rot=tf.transformations.quaternion_from_euler(-eu[1],0,0)
    t.header.stamp = data.header.stamp
    
    t.transform.rotation.x = rot[0]
    t.transform.rotation.y = rot[1]
    t.transform.rotation.z = rot[2]
    t.transform.rotation.w = rot[3]

    cloud_out = do_transform_cloud(data, t)
    np_corrected=ros_numpy.numpify(cloud_out)
    corrected=np_corrected.reshape(np_data.shape)

    img= np.copy(corrected['y'])

    img[np.isnan(img)]=2
    img3 = np.where((img>low)&(img< 0.99*(trans[2])),img,255)
    return img3

def plane_seg_square_imgs(lower=500 ,higher=50000,reg_ly= 30,reg_hy=600,plt_images=True):

    #Segment  Plane using corrected point cloud
    #Lower, higher = min, max area of the box
    #reg_ly= 30,reg_hy=600    Region (low y  region high y ) Only centroids within region are accepted
    
    image= rgbd.get_h_image()
    iimmg= rgbd.get_image()
    points_data= rgbd.get_points()
    img=np.copy(image)
    img3= correct_points()


    _,contours, hierarchy = cv2.findContours(img3.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    points=[]
    images=[]
    for i, contour in enumerate(contours):

        area = cv2.contourArea(contour)

        if area > lower and area < higher :
            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])


            boundRect = cv2.boundingRect(contour)
            #just for drawing rect, dont waste too much time on this
            image_aux= iimmg[boundRect[1]:boundRect[1]+max(boundRect[2],boundRect[3]),boundRect[0]:boundRect[0]+max(boundRect[2],boundRect[3])]
            images.append(image_aux)
            img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (0,0,0), 2)
            # calculate moments for each contour
            if (cY > reg_ly and cY < reg_hy  ):

                cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
                cv2.putText(img, "centroid_"+str(i)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
                print ('cX,cY',cX,cY)
                xyz=[]


                for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
                    for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
                        aux=(np.asarray((points_data['x'][ix,jy],points_data['y'][ix,jy],points_data['z'][ix,jy])))
                        if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                            'reject point'
                        else:
                            xyz.append(aux)

                xyz=np.asarray(xyz)
                cent=xyz.mean(axis=0)
                cents.append(cent)
                print (cent)
                points.append(xyz)
            else:
                print ('cent out of region... rejected')
    sub_plt=0
    if plt_images:
        for image in images:

            sub_plt+=1
            ax = plt.subplot(5, 5, sub_plt )

            plt.imshow(image)
            plt.axis("off")

    cents=np.asarray(cents)
    ### returns centroids found and a group of 3d coordinates that conform the centroid
    return(cents,np.asarray(points), images)
def seg_square_imgs(lower=2000,higher=50000,reg_ly=0,reg_hy=1000,reg_lx=0,reg_hx=1000,plt_images=False): 

    #Using kmeans for image segmentation find
    #Lower, higher = min, max area of the box
    #reg_ly= 30,reg_hy=600,reg_lx=0,reg_hx=1000,    Region (low  x,y  region high x,y ) Only centroids within region are accepted
    image= rgbd.get_h_image()
    iimmg= rgbd.get_image()
    points_data= rgbd.get_points()
    values=image.reshape((-1,3))
    values= np.float32(values)
    criteria= (  cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER  ,1000,0.1)
    k=6
    _ , labels , cc =cv2.kmeans(values , k ,None,criteria,30,cv2.KMEANS_RANDOM_CENTERS)
    cc=np.uint8(cc)
    segmented_image= cc[labels.flatten()]
    segmented_image=segmented_image.reshape(image.shape)
    th3 = cv2.adaptiveThreshold(segmented_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    kernel = np.ones((5,5),np.uint8)
    im4=cv2.erode(th3,kernel,iterations=4)
    plane_mask=points_data['z']
    cv2_img=plane_mask.astype('uint8')
    img=im4
    _,contours, hierarchy = cv2.findContours(im4.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    points=[]
    images=[]
    for i, contour in enumerate(contours):

        area = cv2.contourArea(contour)

        if area > lower and area < higher :
            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])


            boundRect = cv2.boundingRect(contour)
            #just for drawing rect, dont waste too much time on this
            image_aux= iimmg[boundRect[1]:boundRect[1]+max(boundRect[3],boundRect[2]),boundRect[0]:boundRect[0]+max(boundRect[3],boundRect[2])]
            images.append(image_aux)
            img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (0,0,0), 2)
            #img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+max(boundRect[2],boundRect[3]), boundRect[1]+max(boundRect[2],boundRect[3])), (0,0,0), 2)
            # calculate moments for each contour
            if (cY > reg_ly and cY < reg_hy and  cX > reg_lx and cX < reg_hx   ):

                cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
                cv2.putText(img, "centroid_"+str(i)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
                print ('cX,cY',cX,cY)
                xyz=[]


                for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
                    for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
                        aux=(np.asarray((points_data['x'][ix,jy],points_data['y'][ix,jy],points_data['z'][ix,jy])))
                        if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                            'reject point'
                        else:
                            xyz.append(aux)

                xyz=np.asarray(xyz)
                cent=xyz.mean(axis=0)
                cents.append(cent)
                print (cent)
                points.append(xyz)
            else:
                print ('cent out of region... rejected')
                images.pop()
    sub_plt=0
    if plt_images:
        for image in images:

            sub_plt+=1
            ax = plt.subplot(5, 5, sub_plt )

            plt.imshow(image)
            plt.axis("off")

    cents=np.asarray(cents)
    images.append(img)
    return(cents,np.asarray(points), images)


def gaze_point(x,y,z):

    ###Moves head to make center point of rgbd image th coordinates w.r.t.map
    ### To do: (Start from current pose  instead of always going to neutral first )
    
    
    
    head_pose = head.get_current_joint_values()
    head_pose[0]=0.0
    head_pose[1]=0.0
    head.set_joint_value_target(head_pose)
    head.go()
    
    trans , rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) #
    
  #  arm_pose=arm.get_current_joint_values()
  #  arm_pose[0]=.1
  #  arm_pose[1]= -0.3
  #  arm.set_joint_value_target(arm_pose)
  #  arm.go()
    
    e =tf.transformations.euler_from_quaternion(rot)
    print('i am at',trans,np.rad2deg(e)[2])
    print('gaze goal',x,y,z)
    #tf.transformations.euler_from_quaternion(rot)


    x_rob,y_rob,z_rob,th_rob= trans[0], trans[1] ,trans[2] ,  e[2]


    D_x=x_rob-x
    D_y=y_rob-y
    D_z=z_rob-z

    D_th= np.arctan2(D_y,D_x)
    print('relative to robot',(D_x,D_y,np.rad2deg(D_th)))

    pan_correct= (- th_rob + D_th + np.pi) % (2*np.pi)

    if(pan_correct > np.pi):
        pan_correct=-2*np.pi+pan_correct
    if(pan_correct < -np.pi):
        pan_correct=2*np.pi+pan_correct

    if ((pan_correct) > .5 * np.pi):
        print ('Exorcist alert')
        pan_correct=.5*np.pi
    head_pose[0]=pan_correct
    tilt_correct=np.arctan2(D_z,np.linalg.norm((D_x,D_y)))

    head_pose [1]=-tilt_correct
    
    
    
    head.set_joint_value_target(head_pose)
    succ=head.go()
    return succ


def move_base(goal_x,goal_y,goal_yaw,time_out=10):

    #using nav client and toyota navigation go to x,y,yaw
    #To Do: PUMAS NAVIGATION
    pose = PoseStamped()
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = "map"
    pose.pose.position = Point(goal_x, goal_y, 0)
    quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
    pose.pose.orientation = Quaternion(*quat)


    # create a MOVE BASE GOAL
    goal = MoveBaseGoal()
    goal.target_pose = pose

    # send message to the action server
    navclient.send_goal(goal)

    # wait for the action server to complete the order
    navclient.wait_for_result(timeout=rospy.Duration(time_out))

    # print result of navigation
    action_state = navclient.get_state()
    return navclient.get_state()


def static_tf_publish(cents):
    ## Publish tfs of the centroids obtained w.r.t. head sensor frame and references them to map (static)
    trans , rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))
    for  i ,cent  in enumerate(cents):
        x,y,z=cent
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            print('nan')
        else:
            broadcaster.sendTransform((x,y,z),rot, rospy.Time.now(), 'Object'+str(i),"head_rgbd_sensor_link")
            rospy.sleep(.2)
            xyz_map,cent_quat= listener.lookupTransform('/map', 'Object'+str(i),rospy.Time(0))
            map_euler=tf.transformations.euler_from_quaternion(cent_quat)
            rospy.sleep(.2)
            static_transformStamped = TransformStamped()
            

            ##FIXING TF TO MAP ( ODOM REALLY)    
            #tf_broadcaster1.sendTransform( (xyz[0],xyz[1],xyz[2]),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time(0), "obj"+str(ind), "head_rgbd_sensor_link")
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "map"
            
            static_transformStamped.transform.translation.x = float(xyz_map[0])
            static_transformStamped.transform.translation.y = float(xyz_map[1])
            static_transformStamped.transform.translation.z = float(xyz_map[2])
            #quat = tf.transformations.quaternion_from_euler(-euler[0],0,1.5)
            static_transformStamped.transform.rotation.x = 0#-quat[0]#trans.transform.rotation.x
            static_transformStamped.transform.rotation.y = 0#-quat[1]#trans.transform.rotation.y
            static_transformStamped.transform.rotation.z = 0#-quat[2]#trans.transform.rotation.z
            static_transformStamped.transform.rotation.w = 1#-quat[3]#trans.transform.rotation.w
            if xyz_map[2] > .7 and xyz_map[2] < .85:
                static_transformStamped.child_frame_id = "Object_"+str(i)+"_Table_real_lab"
                tf_static_broadcaster.sendTransform(static_transformStamped)
                print xyz_map[2]
            
            
            if xyz_map[2] > .4 and xyz_map[2] < .46:   #table 1 
                static_transformStamped.child_frame_id = "Object_"+str(i)+"_Table_1"
                tf_static_broadcaster.sendTransform(static_transformStamped)
                print xyz_map[2]
            if  xyz_map[2] < .25:   #Floor
                static_transformStamped.child_frame_id = "Object_"+str(i)+"_Floor"
                tf_static_broadcaster.sendTransform(static_transformStamped)
                print xyz_map[2]
    return True
    

def move_d_to(target_distance=0.5,target_link='Floor_Object0'):
    ###Face towards Targetlink and get target distance close

    obj_tar,_ =  listener.lookupTransform('map',target_link,rospy.Time(0))
    robot, _ =  listener.lookupTransform('map','base_link',rospy.Time(0))
    pose, quat =  listener.lookupTransform('base_link',target_link,rospy.Time(0))

    D=np.asarray(obj_tar)-np.asarray(robot)
    d=D/np.linalg.norm(D)
    if target_distance==-1:
        new_pose=np.asarray(robot)
    else:
        new_pose=np.asarray(obj_tar)-target_distance*d
    
    broadcaster.sendTransform(new_pose,(0,0,0,1), rospy.Time.now(), 'D_from_object','map')
    wb_v= whole_body.get_current_joint_values()

    arm.set_named_target('go')
    arm.go()


    succ=move_base( new_pose[0],new_pose[1],         np.arctan2(pose[1],pose[0])+wb_v[2])
    return succ   
        

    ##### Define state INITIAL #####
#Estado inicial de takeshi, neutral
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0

        
    def execute(self,userdata):

        
        rospy.loginfo('STATE : robot neutral pose')
        print('Try',self.tries,'of 5 attepmpts') 
        self.tries+=1
        if self.tries==3:
            return 'tries'
        clear_octo_client()
        scene.remove_world_object()
        #Takeshi neutral
        arm.set_named_target('go')
        arm.go()
        head.set_named_target('neutral')
        succ = head.go()             
        if succ:
            return 'succ'
        else:
            return 'failed'

class Scan_floor(smach.State):### check next state's goal 
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : Scan Floor ')
        move_base(1.21,0.1,0.0)
        gaze_point(1.0,1.0,0.1)
        #cents,xyz, images=seg_square_imgs(plt_images=True)
        cents,xyz, images=plane_seg_square_imgs(plt_images=True)
        succ=True
        if len (cents) != 0:

            succ=static_tf_publish(cents)
        gaze_point(1.2,1.2,0.1)
        #cents,xyz, images=seg_square_imgs(plt_images=True)
        cents,xyz, images=plane_seg_square_imgs(plt_images=True)
        succ=True
        if len (cents) != 0:

            succ=static_tf_publish(cents)



        self.tries+=1
        if self.tries==3:
            self.tries=0 
            return'tries'
        if succ:
            return 'succ'
        else:
            return 'failed'
       
##################################################Pre_grasp_floor()      
class Pre_grasp_floor(smach.State):###example of a state definition.
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : PRE_GRASP_FLOOR')
        
        target_tf= 'Object_0_Floor'
        move_d_to(0.7,target_tf)
        arm.set_named_target('neutral')
        arm.go()
        head.set_named_target('neutral')
        head.go()


        wb_p = whole_body.get_current_pose()
        pose,quat= listener.lookupTransform(  '/base_link',target_tf,rospy.Time(0))
        pose[0]+=-0.3
        #pose[1]+= 0.05
        broadcaster.sendTransform(pose, quat,rospy.Time.now(),'Pre_grasp','base_link')
        rospy.sleep(.1)    
        pose[0]+= 0.23
        broadcaster.sendTransform(pose, quat,rospy.Time.now(),'Grasp','base_link')
        rospy.sleep(.1)    
        try:
            xyz_map, quat =  listener.lookupTransform('map','Pre_grasp',rospy.Time(0))
        except(tf.LookupException):
            print ('no pre grasp table1 tf')
        try:
            xyz_map_grasp, _ =  listener.lookupTransform('map','Grasp',rospy.Time(0))
        except(tf.LookupException):
            print ('no grasp table1 tf')
        clear_octo_client()
        
        #whole_body.set_joint_value_target(wb_give_object)
        #whole_body.go()
        
        wb_p=whole_body.get_current_pose()

        wb_t=wb_p
        wb_t.pose.position.x=  xyz_map[0]
        wb_t.pose.position.y= xyz_map[1]
        wb_t.pose.position.z=0.035
        whole_body.set_pose_target(wb_t)
        
        replan =0
        while replan < 10:
            wb_t.pose.position.x=  xyz_map[0] + 0.05*replan
            wb_t.pose.position.y= xyz_map[1]  + 0.05*replan  
            wb_t.pose.position.z=0.035          + 0.05*replan  
            whole_body.set_pose_target(wb_t)
            print wb_t
            plan=whole_body.plan()
            
            if len (plan.joint_trajectory.points) != 0:
                break
            else: replan+=1

        

        
        
        succ= False
        if len (plan.joint_trajectory.points) != 0:
            succ= whole_body.go()
        

      

        
        
        


        
        self.tries+=1
        if self.tries==3:
            self.tries=0 
            return'tries'
        if succ:
            return 'succ'
        else:
            return 'failed'
##### Define state SCAN_TABLE #####
class Goto_table(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries','end'],input_keys=['counter_in'],output_keys=['counter_out'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        
        global cents, rot, trans
        
        userdata.counter_out=userdata.counter_in +1
        if (userdata.counter_in>  2 ):
            return 'end'

        #goal_x , goal_y, goal_yaw = kl_table1
        #move_base_goal(goal_x+.25*self.tries, goal_y , goal_yaw)      
        

        goal_x = 1.1
        goal_y = 1.2
        goal_yaw = 1.57
        goal_xyz=np.asarray((goal_x,goal_y,goal_yaw))
        
        

        succ=move_base(goal_x , goal_y , goal_yaw)
        xyz=whole_body.get_current_joint_values()[:3]

        print ('goal is ',goal_xyz,'current',xyz)
        print ('Distance is ' ,np.linalg.norm(xyz-goal_xyz))
        if (np.linalg.norm(xyz-goal_xyz)  < .3):
            rospy.loginfo("Navigation Close Enough.")
            return 'succ'
        if succ:
            return 'succ'
        
        if self.tries==5:
            self.tries=0 
            return'tries'

        else:
            return'failed'


    


class Scan_table(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['counter_in'],output_keys=['counter_out'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        if self.tries==3:
            self.tries=0 
            return'tries'
        global cents, rot, trans
        
        userdata.counter_out=userdata.counter_in +1



        gaze_point(1.2,1.7,.41)

        cents,xyz, images=seg_square_imgs(plt_images=True)
        if len (cents) != 0:
            succ=static_tf_publish(cents)

        if succ:
            return 'succ'
        
        if self.tries==5:
            self.tries=0 
            return'tries'

        else:
            return'failed'


        

        """trans, rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))
                                euler = tf.transformations.euler_from_quaternion(rot)        
                                cents = segment_table()
                                if len (cents)==0:
                                    cents = segment_table2(2)
                                    
                                    
                                                                    
                                if len (cents)==0:
                                    arm.set_named_target('go')
                                    arm.go()
                                    head.set_named_target('neutral')
                                    head.go()
                                    return 'failed'
                                else:
                                    print ('tfs published (not static)')
                                    #static_tf_publish(cents)
                                    self.tries=0 
                                    return 'succ'"""
        
class Goto_person(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['counter_in'],output_keys=['counter_out'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        if self.tries==4:
            self.tries=0 
            return'tries'
        goal_x = 0.6
        goal_y = 3.3
        goal_yaw = 2*1.57
        goal_xyz=np.asarray((goal_x,goal_y,goal_yaw))

        # fill ROS message
        pose = PoseStamped()
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = "map"
        pose.pose.position = Point(goal_x, goal_y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose

        # send message to the action server
        navclient.send_goal(goal)

        # wait for the action server to complete the order
        navclient.wait_for_result(timeout=rospy.Duration(10))

        # print result of navigation
        action_state = navclient.get_state()
        print(action_state)
        xyz=whole_body.get_current_joint_values()[:3]
        rospy.loginfo (str(whole_body.get_current_joint_values()[:2]))
        print ('goal is ',goal_xyz,'current',xyz)
        print ('Distance is ' ,np.linalg.norm(xyz-goal_xyz))
        if (np.linalg.norm(xyz-goal_xyz)  < .3):
            rospy.loginfo("Navigation Close Enough.")
            return 'succ'

        if action_state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation Succeeded.")
            return 'succ'
        else:
            print(action_state)
            return'failed'

class Give_object(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['counter_in'],output_keys=['counter_out'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        

        ###### MOVEIT IT IS A BIT COMPLEX FOR IN CODE COMMENTS; PLEASE CONTACT 
        ######## we are seting all the joints in the "whole body " command group to a known value
        #######  conveniently named give object
        #######   before using  clearing the octomap service might be needed



        clear_octo_client()
        wb_give_object=[0.57, 3.26, 3.10, 0.057,-0.822,-0.0386, -0.724, 0.0, 0.0]
        whole_body.set_joint_value_target(wb_give_object)
        whole_body.go()

        print ('yey')
        return 'succ'






        if self.tries==3:
            self.tries=0 
            return'tries'





        


#Initialize global variables and node
def init(node_name):
    global listener, broadcaster, tfBuffer, tf_static_broadcaster, scene, rgbd  , head,whole_body,arm  ,goal,navclient,clear_octo_client,service_client
    rospy.init_node(node_name)
    head = moveit_commander.MoveGroupCommander('head')
    gripper =  moveit_commander.MoveGroupCommander('gripper')
    whole_body=moveit_commander.MoveGroupCommander('whole_body_weighted')
    arm =  moveit_commander.MoveGroupCommander('arm')
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    whole_body.set_workspace([-6.0, -6.0, 6.0, 6.0]) 
    scene = moveit_commander.PlanningSceneInterface()
    rgbd = RGBD()
    goal = MoveBaseGoal()
    navclient = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
    clear_octo_client = rospy.ServiceProxy('/clear_octomap', Empty)
    service_client = rospy.ServiceProxy('/segment_2_tf', Trigger)
    #service_client.wait_for_service(timeout=1.0)
   

    
    
  

#Entry point    
if __name__== '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    sm.userdata.sm_counter = 0

    with sm:
        #State machine for grasping on Floor
        smach.StateMachine.add("INITIAL",       Initial(),      transitions = {'failed':'INITIAL',      'succ':'SCAN_FLOOR',    'tries':'END'}) 
        smach.StateMachine.add("SCAN_FLOOR",    Scan_floor(),      transitions = {'failed':'INITIAL',      'succ':'PRE_GRASP_FLOOR',    'tries':'INITIAL'}) 
        smach.StateMachine.add("PRE_GRASP_FLOOR",   Pre_grasp_floor() ,      transitions = {'failed':'PRE_GRASP_FLOOR',      'succ':'END',    'tries':'INITIAL'}) 
        

        smach.StateMachine.add("GOTO_TABLE",    Goto_table(),   transitions = {'failed':'GOTO_TABLE',   'succ':'SCAN_TABLE',     'tries':'INITIAL', 'end':'END'},remapping={'counter_in':'sm_counter','counter_out':'sm_counter'})
        smach.StateMachine.add("SCAN_TABLE",    Scan_table(),   transitions = {'failed':'SCAN_TABLE',   'succ':'END',     'tries':'INITIAL'},remapping={'counter_in':'sm_counter','counter_out':'sm_counter'})
        smach.StateMachine.add("GOTO_PERSON",    Goto_person(),   transitions = {'failed':'GOTO_PERSON',   'succ':'GIVE_OBJECT',     'tries':'INITIAL'},remapping={'counter_in':'sm_counter','counter_out':'sm_counter'})
        smach.StateMachine.add("GIVE_OBJECT",    Give_object(),   transitions = {'failed':'GIVE_OBJECT',   'succ':'END',     'tries':'INITIAL'},remapping={'counter_in':'sm_counter','counter_out':'sm_counter'})
        

        

      

    outcome = sm.execute()


    