    #rospy.wait_for_service('go_to_target')
    #rospy.wait_for_service('get_pos')

    global X, Y, Z, depth_done
    A = [1.5, 1.5, 1.5]
    B = [1.1, 1.1, 1.1]

    #goto = rospy.ServiceProxy('go_to_target',GoToTarget)
    #getpos = rospy.ServiceProxy("get_pos",GetPos)

    print('Setup done in master node')
    i = 0
    pub_BP.publish(False)
    while not rospy.is_shutdown():
        if i == 0:
            #responseA = goto(A[0], A[1], A[2])
            print('Moving to A and running dnn')
            pub_BP.publish(True)
            while not depth_done:
                print depth_done, right_pos, X
                time.sleep(0.5)
            #responsespot = goto(X, Y, Z)
            depth_done = False
            print('Reached target, planting seedling---------')
            time.sleep(2)
            print('Planting done, next phase')
            i = 1

        if i == 1:
            #responseB = goto(B[0], B[1], B[2])
            pub_BP.publish(True)
            print('Moving to A and running dnn')
            while not depth_done:
                time.sleep(0.5)
            #responsespot = goto(X, Y, Z)
            depth_done = False
            print('Reached target, planting seedling---------')
            time.sleep(2)
            print('Planting done, next phase')
            i = 0


try:
        resp1 = add_two_ints(x, y)
       publisher.publish(resp1)
    except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))


def add_two_ints_client(req):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(req)
        return resp1.X, resp1.Y, resp1.Z
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
