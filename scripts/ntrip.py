#!/usr/bin/env python
'''
two receiver DGPS test code
'''

import ublox, sys, time, struct
import ephemeris, util
import RTCMv3_decode
import math

import rospy
from sensor_msgs.msg import NavSatFix    # ROS message form
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped

from optparse import OptionParser

i_navsatfix = [0,0] # sequence number of navsatfix
i_odometry = [0,0]  # sequence number of odometry
i_relpos = [0,0]  # sequence number of odometry

fix_status = [0,0]
fix_tag =["float", "fix"]




rospy.init_node("ublox_node")

# ROS publisher initialize
pub_leftnavsatfix = rospy.Publisher("left_gnss/fix", NavSatFix, queue_size = 2)
pub_leftvel = rospy.Publisher('left_gnss/fix_vel', TwistWithCovarianceStamped, queue_size = 2)
pub_leftrelpos = rospy.Publisher("left_gnss/relpos", Odometry, queue_size=2)

parser = OptionParser("dgps_test.py [options]")
parser.add_option("--port1", help="serial port 1", default='/dev/serial/by-path/pci-0000:00:14.0-usb-0:10.1.3:1.0')
parser.add_option("--baudrate", type='int', help="serial baud rate", default=115200)
parser.add_option("--log1", help="log file1", default=None)
parser.add_option("--reopen", action='store_true', default=False, help='re-open on failure')
parser.add_option("--dynmodel1", type='int', default=ublox.DYNAMIC_MODEL_PEDESTRIAN, help="dynamic model for recv1")

parser.add_option("--ntrip-server", default="RTK2go.com")
parser.add_option("--ntrip-port", type='int', default=2101)
parser.add_option("--ntrip-user", default="")
parser.add_option("--ntrip-password", default = "")
parser.add_option("--ntrip-mount", default = "HOSEI-RTCM3")

(opts, args) = parser.parse_args()

def setup_port(port1, log1, append=False):
    if (port1!=None):
        dev1 = ublox.UBlox(port1, baudrate=opts.baudrate, timeout=0.1)
        #dev1.set_preferred_dynamic_model(opts.dynmodel1)
        #dev1.set_logfile(log1, append=append)
        #dev1.set_binary()
        #dev1.configure_poll_port()
        #dev1.configure_poll(ublox.CLASS_CFG, ublox.MSG_CFG_USB)
        #dev1.configure_poll(ublox.CLASS_CFG, ublox.MSG_CFG_NAVX5)
        #dev1.configure_poll(ublox.CLASS_MON, ublox.MSG_MON_HW)
        #dev1.configure_poll(ublox.CLASS_NAV, ublox.MSG_NAV_DGPS)
        #dev1.configure_poll(ublox.CLASS_MON, ublox.MSG_MON_VER)
        #dev1.configure_port_USB()
        #dev1.configure_solution_rate(rate_ms=200)
        #dev1.configure_port(port=ublox.PORT_USB, inMask=32, outMask=1)
        #dev1.configure_loadsave(clearMask=0, saveMask=7967, loadMask=0, deviceMask=23)
        #dev1.configure_poll_port()
        #dev1.configure_poll_port(ublox.PORT_USB)
        return dev1
    else:
        print("localization will be done with a device.")
        return dev1
    



errlog = open(time.strftime('errlog-%y%m%d-%H%M.txt'), mode='w')
errlog.write("normal DGPS normal-XY DGPS-XY\n")

def display_diff(name, pos1, pos2):
    print("%13s err: %6.2f errXY: %6.2f pos=%s" % (name, pos1.distance(pos2), pos1.distanceXY(pos2), pos1.ToLLH()))

'''
def handle_device2(msg):
    global rx2_pos
    handle message from rover GPS
    if msg.name() == 'NAV_DGPS':
        msg.unpack()
        print("DGPS: age=%u numCh=%u" % (msg.age, msg.numCh))
    if msg.name() == "NAV_POSECEF":
        msg.unpack()
        rx2_pos = util.PosVector(msg.ecefX*0.01, msg.ecefY*0.01, msg.ecefZ*0.01)

        print("-----------------")
        display_diff("REF<->RECV2", rx2_pos, reference_position)
        
        if dev3 is not None and rx3_pos is not None:
            display_diff("REF<->RECV3", reference_position, rx3_pos)
            errlog.write("%f %f %f %f\n" % (
                reference_position.distance(rx3_pos),
                reference_position.distance(rx2_pos),
                reference_position.distanceXY(rx3_pos),
                reference_position.distanceXY(rx2_pos)))
            errlog.flush()
'''

def handle_device(msg, num):
    '''handle message from uncorrected rover GPS'''
    if msg.name() == "NAV_HPPOSLLH":
        msg.unpack()
        publish_navsatfix(msg, num)
        return
        
    elif msg.name() == "NAV_VELNED":
        msg.unpack()
        publish_velocity(msg, num)
        return

    elif msg.name() == "RXM_RTCM":
        msg.unpack()
        #print("RTCM status is " + str(msg.flag))

    if msg.name() == 'NAV_DGPS':
        msg.unpack()
        print("DGPS: age=%u numCh=%u" % (msg.age, msg.numCh))

    elif msg.name() == "NAV_PVT":
        msg.unpack()
        set_fix_status(msg, num)
    elif msg.name() == 'MON_MSGPP':
        msg.unpack()
        #print("msg:   usb_port:%u, %u, %u, %u, %u, %u, %u, %u" % (msg.msg4[0], msg.msg4[1], msg.msg4[2], msg.msg4[3], msg.msg4[4], msg.msg4[5], msg.msg4[6], msg.msg4[7]))
        #print("       skipped:%u, %u, %u, %u, %u, %u" % (msg.skipped[0], msg.skipped[1], msg.skipped[2], msg.skipped[3], msg.skipped[4], msg.skipped[5]))

    elif msg.name() == 'NAV_RELPOSNED':
        msg.unpack()
        publish_relpos(msg, num)
                
    return

def publish_navsatfix(msg,num):

    global i_navsatfix
    
    if msg.name() != 'NAV_HPPOSLLH':
        print "error: invalid type of message!!!!"
        return
        
    navsatfix = NavSatFix()
    navsatfix.header.stamp = rospy.Time.now()
    if num == 1:
        navsatfix.header.frame_id = "/left_gnss"
    
    elif num == 2:
        navsatfix.header.frame_id = "/right_gnss"
        
    navsatfix.header.seq = i_navsatfix[num-1]
    navsatfix.latitude = msg.Latitude * 1e-7 + msg.latHp * 1e-9
    navsatfix.longitude = msg.Longitude * 1e-7 + msg.lonHp * 1e-9
    navsatfix.altitude = msg.height * 1e-3 + msg.heightHp * 1e-4
    navsatfix.position_covariance = [(msg.hAcc*1e-4)/math.sqrt(2.), 0, 0, 0, (msg.hAcc*1e-4)/math.sqrt(2.), 0, 0, 0, (msg.vAcc*1e-4)]
    navsatfix.status.status = fix_status[num-1]
    
    if num == 1:
        pub_leftnavsatfix.publish(navsatfix)
        
    elif num == 2:
        pub_rightnavsatfix.publish(navsatfix)
    
    i_navsatfix[num-1] +=1
    

def publish_velocity(msg,num):

    global i_odometry
    
    if msg.name() != 'NAV_VELNED':
        print "error: invalid type of message!!"
        return
        
    odometry =TwistWithCovarianceStamped()
    odometry.header.stamp = rospy.Time.now()
    if num == 1:
        odometry.header.frame_id = "/left_gnss"
    
    elif num == 2:
        odometry.header.frame_id = "/right_gnss"
        
    odometry.header.seq = i_odometry[num-1]
    odometry.twist.twist.linear.x = msg.velE * 1e-2
    odometry.twist.twist.linear.y = msg.velN * 1e-2
    odometry.twist.twist.linear.z = -msg.velD * 1e-2
    odometry.twist.covariance = [msg.sAcc * 1e-2/math.sqrt(3.), 0., 0., 0., 0., 0., 
                                0., msg.sAcc * 1e-2/math.sqrt(3.), 0., 0., 0., 0., 
                                0., 0., msg.sAcc * 1e-2/math.sqrt(3.), 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.]
    
    if num == 1:
        pub_leftvel.publish(odometry)
        
    elif num == 2:
        pub_rightvel.publish(odometry)
    
    i_odometry[num-1] +=1


def publish_relpos(msg,num):

    global i_relpos
    if msg.name() != 'NAV_RELPOSNED':
        print "error: invalid type of message!!"
        return
        
    odometry = Odometry()
    odometry.header.stamp = rospy.Time.now()
    if num == 1:
        odometry.header.frame_id = "/left_gnss"
    
    elif num == 2:
        odometry.header.frame_id = "/right_gnss"
        
    odometry.header.seq = i_odometry[num-1]
    odometry.pose.pose.position.x = msg.relPosE * 1e-2 + msg.relPosHPE * 1e-4
    odometry.pose.pose.position.y = msg.relPosN * 1e-2 + msg.relPosHPN * 1e-4
    odometry.pose.pose.position.z = -msg.relPosD * 1e-2 -msg.relPosHPD * 1e-4
    odometry.pose.covariance = [msg.accE * 1e-4, 0., 0., 0., 0., 0., 
                                0., msg.accN * 1e-4, 0., 0., 0., 0., 
                                0., 0., msg.accD * 1e-4, 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.]
    
    if num == 1:
        pub_leftrelpos.publish(odometry)
        
    elif num == 2:
        pub_rightrelpos.publish(odometry)
    
    i_relpos[num-1] +=1


def set_fix_status(msg, num):
    global fix_status
    bitfield = msg.flags
    if bitfield >= 2**7:
        fix_status[num-1]=1
    else:
        fix_status[num-1]=0
    
    string = "fix status: left :" + (fix_tag[fix_status[0]]) + ", right: " + (fix_tag[fix_status[1]])
    #print(string)
    
                                                
def send_rtcm(msg):
    import binascii
    print(binascii.b2a_hex(msg))
    try:
        dev1.write(msg)
    except:
        pass
    
    
def receiver_thread(dev, num=1):
    while not rospy.is_shutdown():
        try:
            # get a message from the reference GPS
            msg = dev.receive_message_noerror()
            if msg is not None:
                handle_device(msg, num)
                last_msg_time = time.time()
            sys.stdout.flush()
        except:
            pass

def run_receiver_thread(dev, num):
    import threading

    t = threading.Thread(target=receiver_thread, args=(dev, num))
    t.start()

dev1 = setup_port(opts.port1, opts.log1)

  #RTCMv3_decode.run_RTCM_converter(opts.ntrip_server, opts.ntrip_port, opts.ntrip_user, opts.ntrip_password, opts.ntrip_mount, rtcm_callback=send_rtcm)
RTCMv3_decode.run_RTCM_converter('rtk2go.com', 2101, '', '', 'TSUKUBA-RTCM3', rtcm_callback=send_rtcm)
#RTCMv3_decode.run_RTCM_converter('160.16.134.72', 2101, 'guest', 'guest', 'CQ-RTCM3', rtcm_callback=send_rtcm)

run_receiver_thread(dev1, 1)
#run_receiver_thread(dev2, 2)


