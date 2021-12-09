#!/usr/bin/python

import os
import datetime
import subprocess
import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
# import nav2point
soundhandle = SoundClient()
# nav = nav2point.NavToPoint()
# target_pose = [0,0,0]
# nav.stop()
# nav.goto(target_pose, True)
base_pose = [0, 0, 0]
human_pose = [1.81, 8.38, 0]
animal_pose = [-7.29, 2.06, 0]
plant_pose = [-10.37, 7.95, 0]
fungi_pose = [-23.31, 5.19, 0]
bacteria_pose = [-33.88, 11.06, 0]
virus_pose = [-30.96, 5.33, 0]


def intersection(lst1, lst2):
    return list(set(lst1) & set(lst2))


def contain(lst1, lst2):
    inter = intersection(lst1, lst2)
    # print(inter)
    return len(inter)


def handle_output(data):
    """Map out grammar recognized commands in speech to terminal commands"""
    text_data = data.data.lower().split()
    print(text_data)
    if contain(["oh","all"], text_data):
        if contain(["goto", "go", "to", 'navigate', "take", "bring"], text_data):
            # print("goto")
            if contain(["workspace"], text_data):
                soundhandle.stopAll()
                soundhandle.say('Right away, master!')
                package_path = subprocess.run(
                    ['rospack', 'find', 'museum_nav_bot'], stdout=subprocess.PIPE).stdout.decode('utf-8')
                os.system("nautilus --browser "+package_path)
            elif contain(["base", "home", "exit"], text_data):
                soundhandle.stopAll()
                soundhandle.say('Follow me, master!')
                rospy.loginfo("Goto Home.")
                NavOut = subprocess.run(
                    ['rosrun', 'museum_nav_bot', 'nav2point.py', str(base_pose[0]), str(base_pose[1]), str(base_pose[2])], stdout=subprocess.PIPE).stdout.decode('utf-8')
                rospy.loginfo(NavOut)
            elif contain(["humans","human", "man", "woman","bipedal","civilized","mortal"], text_data):
                soundhandle.stopAll()
                soundhandle.say('Follow me, master!')
                rospy.loginfo("Goto Human zone.")
                NavOut = subprocess.run(
                    ['rosrun', 'museum_nav_bot', 'nav2point.py', str(human_pose[0]), str(human_pose[1]), str(human_pose[2])], stdout=subprocess.PIPE).stdout.decode('utf-8')
                rospy.loginfo(NavOut)
            elif contain(["animals","animal", "beastly","beast","mammal"], text_data):
                soundhandle.stopAll()
                soundhandle.say('Follow me, master!')
                rospy.loginfo("Goto Animal zone.")
                NavOut = subprocess.run(
                    ['rosrun', 'museum_nav_bot', 'nav2point.py', str(animal_pose[0]), str(animal_pose[1]), str(animal_pose[2])], stdout=subprocess.PIPE).stdout.decode('utf-8')
                rospy.loginfo(NavOut)
            elif contain(["plant", "tree", "green","wood","forest","sapling","seedling"], text_data):
                soundhandle.stopAll()
                soundhandle.say('Follow me, master!')
                rospy.loginfo("Goto Plant zone.")
                NavOut = subprocess.run(
                    ['rosrun', 'museum_nav_bot', 'nav2point.py', str(plant_pose[0]), str(plant_pose[1]), str(plant_pose[2])], stdout=subprocess.PIPE).stdout.decode('utf-8')
                rospy.loginfo(NavOut)
            elif contain(["fungi", "fungus", "mushroom","yeast"], text_data):
                soundhandle.stopAll()
                soundhandle.say('Follow me, master!')
                rospy.loginfo("Goto Fungus zone.")
                NavOut = subprocess.run(
                    ['rosrun', 'museum_nav_bot', 'nav2point.py', str(fungi_pose[0]), str(fungi_pose[1]), str(fungi_pose[2])], stdout=subprocess.PIPE).stdout.decode('utf-8')
                rospy.loginfo(NavOut)
            elif contain(["bacteria", "germs","microbes"], text_data):
                soundhandle.stopAll()
                soundhandle.say('Follow me, master!')
                rospy.loginfo("Goto Bacteria zone.")
                NavOut = subprocess.run(
                    ['rosrun', 'museum_nav_bot', 'nav2point.py', str(bacteria_pose[0]), str(bacteria_pose[1]), str(bacteria_pose[2])], stdout=subprocess.PIPE).stdout.decode('utf-8')
                rospy.loginfo(NavOut)
            elif contain(["virus"], text_data):
                soundhandle.stopAll()
                soundhandle.say('Follow me, master!')
                rospy.loginfo("Goto Virus zone.")
                NavOut = subprocess.run(
                    ['rosrun', 'museum_nav_bot', 'nav2point.py', str(virus_pose[0]), str(virus_pose[1]), str(virus_pose[2])], stdout=subprocess.PIPE).stdout.decode('utf-8')
                rospy.loginfo(NavOut)
            else:
                soundhandle.stopAll()
                soundhandle.say("sory, i can\'t hear the location propery")

    elif contain(["hello", "hi"], text_data):
        soundhandle.stopAll()
        soundhandle.say('hello master')
    elif "where is avenger base" in data.data.lower():
        soundhandle.stopAll()
        soundhandle.say('I am sorry! That is classified information')
    elif "what time is it" in data.data.lower():
        message = 'It is ' + datetime.datetime.now().strftime("%H:%M") + ' hours'
        print(message)
        soundhandle.stopAll()
        soundhandle.say(message)
    elif "good night" in data.data.lower():
        message = 'good night, master!'
        soundhandle.stopAll()
        soundhandle.say(message)
        exit()
    else:
        print("un-recognize command:", data.data)


def shutdown():
    """This function is executed on node shutdown."""
    # command executed after Ctrl+C is pressed
    rospy.loginfo("Shutdown command center.")
    rospy.sleep(1)


def init():
    """Initialize node and subscribe to necessary topics"""

    # initialize node
    # rospy.init_node('soundplay_test', anonymous = True)
    rospy.init_node("command_control")

    # Call custom function on node shutdown
    rospy.on_shutdown(shutdown)
    rospy.loginfo("ready to command...")
    rospy.Subscriber("speech_recog_data", String, handle_output)

    rospy.spin()


if __name__ == "__main__":
    init()
