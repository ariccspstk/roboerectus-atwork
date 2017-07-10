#!/usr/bin/python
import rospy
from std_msgs.msg import Empty
import rosparam
# from refereeBoxClient import *
from os import getenv
import subprocess
import socket
import copy
import zmq
import sys

SERVER = "127.0.0.1"
PORT = "11111"
TEAM = "RoboErectus"
MESSAGE = "connection Test"
errorPrinted = False
ON = True  # False
#ON = False

PLAN_FILE = getenv("HOME") + "/ros_ws/src/aricc-ros-pkg/aricc_demo/rcwork2015/aricc_rcwork_task/config/plan/plan_"

def parse_msg(msg):
    test = msg[0:msg.index("<")]
    goals = msg[msg.index("<") + 1:-1]  # remove closing bracket
    #print "\nTEST: %s\n" % test

    if test == "BNT":
        parse_bnt(goals)

    if test == "BMT":
        parse_bmt(goals)

    if test == "BTT":
        parse_btt(goals)

    if test == "PPT":
        parse_ppt(goals)

    if test == "CBT":
        parse_cbt(goals)

def parse_ppt(goals):
    print goals
    src_place  = goals.split(",")[0]
    dest_place = goals[goals.find(")") + 2:]
    objects    = goals[goals.find("(") + 1:goals.find(")")].split(",")
    out = "plans:\n"
    out += output_format(src_place, objects, "PICKUP_TABLE",0, "UNKNOWN")
    out += output_format(dest_place,objects, "PLACE_HOLE",  0, "UNKNOWN")
    out += "\n"
    # End location
    out += output_format('E1', [], "EXIT", 0, "UNKNOWN")
    print out
    print "\nsaving plan to file (%s)...\n" % PLAN_FILE
    write_and_start( PLAN_FILE, out, "ppt" )

def parse_cbt(goals):
    print goals
    src_place = goals
    objects = ['F20_20_B', 'F20_20_G', 'S40_40_B', 'S40_40_G', 'M20_100', 'M20', 'M30', 'R20', 'V20']
    out = "plans:\n"
    out += output_format( src_place, objects, "PICKUP_BELT",0,"UNKNOWN")
    out += "\n"
    # End location
    out += output_format('E1', [], "EXIT", 0, "UNKNOWN")
    print out
    print "\nsaving plan to file (%s)...\n" % PLAN_FILE
    write_and_start(PLAN_FILE, out,"cbt")

def parse_bnt(goals):
    goal_list = goals.split("(")  # split at "("
    del (goal_list[0])  # delete empty first item
    for i, g in enumerate(goal_list):
        goal_list[i] = g[0:-1]  # delete ")"
     
    out = "plans:\n"
    for g in goal_list:
        label, direction, sleep = tuple(g.split(","))
        out += output_format(label, [], "EXIT", sleep, direction)
        out += "\n"
    # End location
    out += output_format('E1', [], "EXIT", 0, "UNKNOWN")
    print out
    print "\nsaving plan to file (%s)...\n" % PLAN_FILE
    write_and_start(PLAN_FILE, out,"bnt")

def parse_bmt(goals):
    print goals
    goal_list   = goals.split(",")
    init_place  = goal_list[0]
    src_place   = goal_list[1]
    dest_place  = goal_list[2]
    cfg         = goal_list[3][0:goal_list[3].find("(")]
    objects     = goals[goals.find("(") + 1:goals.find(")")].split(",")
    final_place = goals[goals.find(")") + 2:len(goals)]

    #print "_____________________________"
    #print "initial: %s" % init_place
    #print "source_place: %s" % src_place
    #print "destination_place: %s" % dest_place
    #print "configuration: %s" % cfg
    #print "objects:" % objects
    #for obj in objects:
    #    print "\t%s" % obj
    #print "final_place: %s" % final_place
    #print "_____________________________"

    out = "plans:\n"
    out += output_format(init_place, [],     "EXIT",        0,"UNKNOWN")
    out += output_format(src_place, objects, "PICKUP_TABLE",0,"UNKNOWN")
    out += output_format(dest_place, objects,"PLACE_TABLE", 0,"UNKNOWN")
    out += output_format(final_place,[],     "EXIT",        0,"UNKNOWN")
    out += "\n"
    # End location
    out += output_format('E1', [], "EXIT", 0, "UNKNOWN")
    print out
    print "\nsaving plan to file (%s)...\n" % PLAN_FILE
    write_and_start(PLAN_FILE, out, "bmt")

def parse_btt(goals):
    print goals
    goals = goals.strip()

    init_goal, dest_goal = goals.split(';')
    init_goal = init_goal.strip()
    dest_goal = dest_goal.strip()
    
    #remove "initialsituation" and "goalsituation"
    init_goal = init_goal[17:-1]
    dest_goal = dest_goal[14:-1]

    init_goal_list = init_goal.split('<')
    del (init_goal_list[0])
    for i, g in enumerate(init_goal_list):
        init_goal_list[i] = g[0:-1]
        #print init_goal_list[i]

    dest_goal_list = dest_goal.split('<')
    del (dest_goal_list[0])
    for i, g in enumerate(dest_goal_list):
        dest_goal_list[i] = g[0:-1]
        #print dest_goal_list[i]

    # Solving init_goal_list
    no_of_init_states = 0
    source_list = []
    for g in init_goal_list:
        place = g[0:g.index(',')]
        config = g[g.index(',') + 1:g.find("(")]
        object_list = g[g.find("(") + 1:g.find(")")]
        objects = object_list.split(',')
        for obj in objects:
            item = []
            item.append(place)
            item.append(obj)
            source_list.append(item)
    #print source_list

    # Goal states
    goal_states = {}
    dest_list = []
    for g in dest_goal_list:
        place = g[0:g.index(',')]
        config = g[g.index(',') + 1:g.find("(")]
        object_list = g[g.find("(") + 1:g.find(")")]
        objects = object_list.split(',')
        
        #merge objects in same place
        if place in goal_states.keys():
            goal_objects = goal_states[place]
            for obj in objects:
                goal_objects.append(obj)
        else:
            goal_objects = []
            for obj in objects:
                goal_objects.append(obj)
            goal_states[place] = goal_objects
    #print dest_list

    # Sorting goal objects 
    three_object_list = []
    two_object_list   = []
    one_object_list   = []
    for (place, goal_objects) in goal_states.items():
        no_of_objects = len(goal_objects)
        if no_of_objects == 4:
            temp_list = []
            temp_list.append(place)
            top_three = []
            top_three.append(goal_objects[0])
            top_three.append(goal_objects[1])
            top_three.append(goal_objects[2])
            temp_list.append(top_three)
            three_object_list.append(temp_list)
            temp_list = []
            temp_list.append(place)
            last_one = []
            last_one.append(goal_objects[3])
            temp_list.append(last_one)
            one_object_list.append(temp_list)
        elif no_of_objects == 3:
            temp_list = []
            temp_list.append(place)
            temp_list.append(goal_objects)
            three_object_list.append(temp_list)
        elif no_of_objects == 2:
            temp_list = []
            temp_list.append(place)
            temp_list.append(goal_objects)
            two_object_list.append(temp_list)
        elif no_of_objects == 1:
            temp_list = []
            temp_list.append(place)
            temp_list.append(goal_objects)
            one_object_list.append(temp_list)
    print "one_object_list: %s" % one_object_list    
    #print "two_object_list: %s" % two_object_list    
    #print "three_object_list: %s" % three_object_list    


    # Group goal objects
    i = 0
    goal_final = {}
    goal_final_ = {}
    group_no = 0
    goal_item_list_ = []
    while len(three_object_list) :
        item_list = []
        obj = three_object_list[0]
        goal_item_list_.append(obj)
        item_list.append(obj)
        goal_final[group_no] = item_list
        group_no += 1
        three_object_list.remove(obj)

    while len(two_object_list) :
        item_list = []
        if len(one_object_list):
            obj = two_object_list[0]
            item_list.append(obj)
            goal_item_list_.append(obj)
            two_object_list.remove(obj)

            obj = one_object_list[0]
            item_list.append(obj)
            goal_item_list_.append(obj)
            one_object_list.remove(obj)

            goal_final[group_no] = item_list
            group_no += 1
        else:
            obj = two_object_list[0]
            item_list.append(obj)
            goal_item_list_.append(obj)
            two_object_list.remove(obj)
            goal_final[group_no] = item_list
            group_no += 1

    while len(one_object_list) :
        item_list = []
        if len(one_object_list) > 2 :
            for i in range(0,3):
                obj = one_object_list[0]
                item_list.append(obj)
                goal_item_list_.append(obj)
                one_object_list.remove(obj)
            goal_final[group_no] = item_list
            group_no += 1
        else:
            while len(one_object_list):
                obj = one_object_list[0]
                item_list.append(obj)
                goal_item_list_.append(obj)
                one_object_list.remove(obj)
            goal_final[group_no] = item_list
            group_no += 1
    #print "goal_item_list: %s\n" % goal_item_list_
    goal_final_ = copy.deepcopy(goal_final)
    #for (k,z) in goal_final.items():
    #    goal_final_[k] = z
    #print "goal_final: %s\n" %  goal_final
    #print "goal_final_: %s\n" %  goal_final_
        
    # Group goal and initial    
    pick_group_ = {}
    #print "\nSorting Initial:"
    n = 0
    while n < len(goal_final):
        place_list = goal_final[n]
        pick_list = []
        while len(place_list):
            place_item = place_list[0]
            place_objs = place_item[1]
            while len(place_objs) :
                l = place_objs[0]
                for item in source_list:
                    if item[1] == l:
                        pick_list.append(item)
                        #print "pick_list adding %s" % item
                        source_list.remove(item)
                        break
                place_objs.remove(l)              
            place_list.remove(place_item)            
        pick_group_[n] = pick_list
        #print "current pick_group %s" % pick_group_[n]
        n += 1
    #print "pick_group_: %s\n " % pick_group_  
    #print "goal_final_ after: %s\n" %  goal_final_
     
    # Combine pick list
    #print "pick final length: %s" % len(pick_group_)
    pick_objs = []
    pick_final_ = {}
    i = 0
    while i<len(pick_group_):
        #print "pick_group_[i] %s" % pick_group_[i]
        pick_objs = pick_group_[i]
        #print pick_objs
        if len(pick_objs) == 1:
            pick_obj_1 = pick_objs[0]
            state_1 = pick_obj_1[0]
            item_new = []
            pick_new = []
            obj_new = []
            item_new.append(state_1)
            obj_new.append(pick_obj_1[1])
            item_new.append(obj_new)
            pick_new.append(item_new)
            pick_final_[i] = pick_new
        elif len(pick_objs) == 2:
            pick_obj_1 = pick_objs[0]
            pick_obj_2 = pick_objs[1]
            state_1 = pick_obj_1[0]
            state_2 = pick_obj_2[0]
            if state_1 == state_2:
                item_new = []
                pick_new = []
                obj_new = []
                item_new.append(state_1)
                obj_new.append(pick_obj_1[1])
                obj_new.append(pick_obj_2[1])
                item_new.append(obj_new)
                pick_new.append(item_new)
                pick_final_[i] = pick_new
            else:
                item_new = []
                pick_new = []
                obj_new = []
                item_e = []
                obj_e = []
                item_new.append(state_1)
                obj_new.append(pick_obj_1[1])
                item_new.append(obj_new)
                pick_new.append(item_new)
                
                item_e.append(state_2)
                obj_e.append(pick_obj_2[1])
                item_e.append(obj_e)
                pick_new.append(item_e)
                pick_final_[i] = pick_new
        elif len(pick_objs) == 3:
            pick_obj_1 = pick_objs[0]
            pick_obj_2 = pick_objs[1]
            pick_obj_3 = pick_objs[2]
            state_1 = pick_obj_1[0]
            state_2 = pick_obj_2[0]
            state_3 = pick_obj_3[0]
            if state_1 == state_2:
                item_new = []
                pick_new = []
                obj_new = []
                item_e = []
                obj_e = []
                item_new.append(state_1)
                obj_new.append(pick_obj_1[1])
                obj_new.append(pick_obj_2[1])
                if state_1 == state_3:
                    obj_new.append(pick_obj_3[1])
                    item_new.append(obj_new)
                    pick_new.append(item_new)
                    pick_final_[i] = pick_new
                else:
                    item_new.append(obj_new)
                    pick_new.append(item_new)
                    item_e.append(state_3)
                    obj_e.append(pick_obj_3[1])
                    item_e.append(obj_e)
                    pick_new.append(item_e)
                    pick_final_[i] = pick_new
            elif state_1 == state_3:
                item_new = []
                obj_new = []
                item_e = []
                obj_e = []
                pick_new = []
                item_new.append(state_1)
                obj_new.append(pick_obj_1[1])
                obj_new.append(pick_obj_3[1])
                item_new.append(obj_new)
                pick_new.append(item_new)
                
                item_e.append(state_2)
                obj_e.append(pick_obj_2[1])
                item_e.append(obj_e)
                pick_new.append(item_e)

                pick_final_[i] = pick_new
            elif state_2 == state_3:
                item_new = []
                pick_new = []
                obj_new = []
                item_e = []
                obj_e = []
                item_new.append(state_2)
                obj_new.append(pick_obj_2[1])
                obj_new.append(pick_obj_3[1])
                item_new.append(obj_new)
                pick_new.append(item_new)
                
                item_e.append(state_1)
                obj_e.append(pick_obj_1[1])
                item_e.append(obj_e)
                pick_new.append(item_e)
                
                pick_final_[i] = pick_new
            else:
                item_new = []
                item_e = []
                item_e_e = []
                obj_new = []
                obj_e = []
                obj_e_e = []
                pick_new = []
                
                item_new.append(state_1)
                obj_new.append(pick_obj_1[1])
                item_new.append(obj_new)
                pick_new.append(item_new)

                item_e.append(state_2)
                obj_e.append(pick_obj_2[1])
                item_e.append(obj_e)
                pick_new.append(item_e)

                item_e_e.append(state_3)
                obj_e_e.append(pick_obj_3[1])
                item_e_e.append(obj_e_e)
                pick_new.append(item_e_e)

                pick_final_[i] = pick_new
        else:
            break
        i += 1
    #print "pick_final_: % s\n" % pick_final_    
    
    # Output
    out = "plans:\n"
    
    j = 0
    #print "goal_final_ b4 out %s" % goal_final_
    while j < len(pick_final_):
        pick = pick_final_[j]
        place = goal_final_[j]
        #print "pick: %s" % pick
        #print "place: %s" % place
        for objs in pick:
            #print "pick objs: %s" % objs
            pick_name = objs[0]
            pick_it = objs[1]
            out += output_format(pick_name, pick_it,"PICKUP_TABLE",0,"UNKNOWN")
        for it in place:
            #print"place objs: %s" %  it
            place_name = it[0]
            place_it = it[1]
            if place_name == "S4":
                action = "PLACE_HOLE"
            else:
                action = "PLACE_TABLE"
            out += output_format(place_name, place_it,action,0,"UNKNOWN")
        j += 1

    # End location
    out += output_format('E1', [], "EXIT", 0, "UNKNOWN")
    print out
    print "\nsaving plan to file (%s)...\n" % PLAN_FILE
    write_and_start(PLAN_FILE, out,"btt")

def output_format(table, objs, obj_type, pause, direction):
    output = "  - {place: '%s', objects: %s, action: '%s', pause: '%s', dir: '%s'}\n" % (table, objs, obj_type, pause, direction)
    return output

def write_and_start(filename, buffer_string, task):
    filename += task + ".yaml"
    print "filename: %s" % filename
    f = open(filename, "w+")
    f.write(buffer_string)
    f.close()
    pub.publish(Empty())

    paramlist = rosparam.load_file(filename)
    for params, ns in paramlist:
        rosparam.upload_params(ns, params)

def obtainTaskSpecFromServer(ServerIP, ServerPort, TeamName):
    # return message
    #while True:
    global errorPrinted
    try:
        context = zmq.Context()
        connection_address = "tcp://" + ServerIP + ":" + ServerPort
        print "Start connection to " + connection_address
        #  Socket to talk to server
        print "Connecting to server..."
        s = context.socket(zmq.REQ)
        s.connect (connection_address)
        errorPrinted = False
    except socket.error, err:
        if not errorPrinted:
            rospy.loginfo("Connection to %s:%s not succsesfull: %s", ServerIP, ServerPort, err)
            rospy.loginfo("retrying ...")
            errorPrinted = True
        rospy.sleep(1.)
    else:
        print "Sending request ..."
        s.send(TeamName)
        waitForData(s)
    rospy.loginfo("Connected to %s : %s", ServerIP, ServerPort)


def waitForData(s):
    rospy.loginfo("Connection Successfull")
    BUFFER_SIZE = 1024
    #while True:
    try:
            #data = s.recv(BUFFER_SIZE)
            data = s.recv()
            print data
            parse_msg(data)
            s.close()
    except socket.error, err:
            rospy.loginfo("Connection closed")
            s.close()
            rospy.sleep(2.)
            return

def testRefbox():
    #goal all three
    #msg = "BTT<initialsituation(<S1,line(1,3)><S4,line(3)><S2,line(2)><S3,line(1,2)>);goalsituation(<D3,line(1,2,3)><S5,line(1,2,3)>)>"
    #goal all two
    #msg = "BTT<initialsituation(<S1,line(1,2)><S4,line(2,3)><S2,line(2)><S3,line(3,2)>);goalsituation(<D3,line(2,3)><S5,line(1,2)>)>"
    #goal all one
    #msg = "BTT<initialsituation(<S1,line(1,2,1)><S4,line(2,3)><S2,line(2)><S3,line(3,2)>);goalsituation(<D3,line(3)><D4,line(1)><S4,line(2)><D4,line(1)>)>"
    #123,1,2
    #msg = "BTT<initialsituation(<S1,line(1,2,1)><S4,line(2,3,1)><S2,line(2)><S3,line(3,2)>);goalsituation(<D3,line(3,2,1)><S4,line(1,2)><D4,line(1)>)>"
    #123,1
    #msg = "BTT<initialsituation(<S1,line(1,2)><S4,line(2,3)><S2,line(2)><S3,line(3,2)>);goalsituation(<D3,line(3,2,1)><D4,line(2)>)>"
    #123,12,1
    #
    #msg = "BTT<initialsituation(<S4,line(M20_100,R20)>);goalsituation(<S3,line(M20_100)><S5,line(R20)>)>"

    #msg = "BTT<initialsituation(<D1,line(M20_100,S40_40_B,F20_20_b)><D2,line(M20_100)><D1,line(F20_20_B,S40_40_B)>);goalsituation(<D3,line(S40_40_B,M20_100,F20_20_B)><D4,line(M20_100)>)>"

    #msg = "BTT<initialsituation(<S1,line(1,2)><S2,line(3,1)><S1,line(3)><S3,line(4,2,3)><S4,line(1,2)>);goalsituation(<D1,line(1)><D1,line(2)><D2,line(3,4)><D3,line(1)><D4,line(1,2,3)><D5,line(2)><D6,line(3)>)>"
    
    #msg = "BTT<initialsituation(<S1,line(1,2)><S2,line(3,1)><S1,line(3)><S3,line(4,2,3)><S4,line(1,2)>);goalsituation(<D1,line(1,2)><D2,line(3,4)><D3,line(1,3)><D4,line(1,2)><D5,line(2,3)>)>"
    
    #msg = "BTT<initialsituation(<S1,line(1,2)><S2,line(1)><S3,line(4,2,3)><S4,line(1,2,3)>);goalsituation(<D1,line(1,2)><D2,line(3)><D3,line(1,3,4)><D4,line(2)><D5,line(2)>)>"
    
    msg = "BTT<initialsituation(<D1,line(M20_100,S40_40_B)><S1,line(M20_100)>);goalsituation(<D1,line(S40_40_B,M20_100)><D2,line(M20_100)>)>"
    #msg = "PPT<D1,(M20_100,F20_20_B,F20_20_B),D2>"
    #msg = "BNT<(S6,N,3)(S2,N,3)(D1,S,3)(S5,W,3)(D3,E,3)(D4,S,3)>"
    #msg = "CBT<D1>"
    #msg = "BMT<S6,S6,S7,line(F20_20_G,F20_20_B,M20_100),S7>"
    #print msg

if __name__ == "__main__":
    rospy.init_node('aricc_refbox')
    pub = rospy.Publisher('/refbox', Empty, queue_size=10)
    obtainTaskSpecFromServer(SERVER, PORT, TEAM)
    #testRefbox()
    #parse_msg(msg)
