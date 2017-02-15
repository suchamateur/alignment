import serial, struct, string, time
import pygame
import numpy as np
import cv2
import ConfigParser
from ponycube import *
from struct import *
import sys
from time import sleep

GYRO_ON = 1
CAMERA_ON = 1

COMPORT1 = 7#2 #sensor 2, on detector
COMPORT2 = 9 #sensor 3, on tube

QUAT_DET = None
QUAT_TUB = None
QUAT_TRANS = Quaternion(1,0,0,0)
DELTA_X = 0.0
DELTA_Y = 0.0

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

FOCAL_LENGTH = 630*3.138;

SPANX = 200
SPANY = 200

counter = 0

class ir_marker:
    def __init__(self):
        #self.pts = [Vector3(-2.035,-2.325,0), Vector3(1.925,-2.3,0), Vector3(2.05,1.77,0), Vector3(-1.71,2.04,0), Vector3(-2.32,-2.3,0), Vector3(2.32,-2.3,0), Vector3(2.32,2.3,0), Vector3(-2.32,2.3,0)]
        #self.pts = [Vector3(-1.85,-2.225,0),Vector3(1.85, -2.225,0),Vector3(1.85,2.225,0),Vector3(-1.85,2.225,0)]
        
        # 90 deg extension
        self.pts = [Vector3(-2.305, 1.915, 0), Vector3(-2.305, -1.915, 0), Vector3(2.305, -1.915, 0), Vector3(2.305, 1.915, 0), Vector3(2.245, 3.605, 0), Vector3(2.245, 4.785, 0)]
        self.axis = [Vector3(1,0,0),Vector3(0,1,0),Vector3(0,0,1)]
        
    def origin(self):
        #self.pts = [Vector3(-2.035,-2.325,0), Vector3(1.925,-2.3,0), Vector3(2.05,1.77,0), Vector3(-1.71,2.04,0), Vector3(-2.32,-2.3,0), Vector3(2.32,-2.3,0), Vector3(2.32,2.3,0), Vector3(-2.32,2.3,0)]
        #self.pts = [Vector3(-1.85,-2.225,0),Vector3(1.85, -2.225,0),Vector3(1.85,2.225,0),Vector3(-1.85,2.225,0)]
        self.pts = [Vector3(-2.305, 1.915, 0), Vector3(-2.305, -1.915, 0), Vector3(2.305, -1.915, 0), Vector3(2.305, 1.915, 0), Vector3(2.245, 3.605, 0), Vector3(2.245, 4.785, 0)]
        self.axis = [Vector3(1,0,0),Vector3(0,1,0),Vector3(0,0,1)]
        
    def rotate(self,q):
        assert isinstance(q, Quaternion)
        R = q.get_matrix()
        self.pts = [R*p for p in self.pts]
        self.axis = [R*p for p in self.axis]
        
    def perspective_transform(self):
        pts = self.pts
        for p in pts:
#             p.z = 0
#             p.x = p.x*50+IMAGE_WIDTH/2
#             p.y = p.y*50+IMAGE_HEIGHT/2
            p.z += 1000
            p.x = 640*p.x/p.z*50+IMAGE_WIDTH/2
            p.y = IMAGE_HEIGHT/2-640*p.y/p.z*50
            p.z = 0
        return pts
    
    def get_perspective_axis(self):
        axis = self.axis
        for p in axis:
            p.z += 1000
            p.x = 640*p.x/p.z*50+IMAGE_WIDTH/2
            p.y = IMAGE_HEIGHT/2-640*p.y/p.z*50
        return axis
    
    def flip(self, pts, flag=0):
        new_pts = pts
        if flag==0:
            for p in new_pts:
                p.y = -p.y
        elif flag>0:
            for p in new_pts:
                p.x = -p.x
        else:
            for p in new_pts:
                p.x = -p.x
                p.y = -p.y
        return pts
        

class packet_reader:

        def __init__(self, port, quat_delegate=None, data_delegate=None ):
            print(port)
            self.s = serial.Serial(port,115200)
            self.s.setTimeout(0.1)
            self.s.setWriteTimeout(0.2)
            self.com = port
            
            if quat_delegate:
                self.quat_delegate = quat_delegate
            else:
                self.quat_delegate = empty_packet_delegate()

            if data_delegate:
                self.data_delegate = data_delegate
            else:
                self.data_delegate = empty_packet_delegate()

            self.packets = []
            self.length = 0
            self.previous = None
            

        def read(self):
            NUM_BYTES = 23

            p = None

            while self.s.inWaiting() >= NUM_BYTES:

                rs = self.s.read(NUM_BYTES)
                
                #sss = ''.join([str(ord(rs[0])),' ']+[str(ord(rs[1])),' ']+[str(ord(a)).ljust(4) for a in [rs[2],rs[3],rs[4],rs[5],rs[6],rs[7],rs[8],rs[9],rs[10]]]+[ str(ord(a)).ljust(4) for a in [rs[8],rs[9],rs[10],rs[11],rs[12],rs[13]]])
                #print (sss)

                if ord(rs[0]) == ord('$'):

                    pkt_code = ord(rs[1])

                    if pkt_code == 1:

                        #print ('code1')
                        pass

                        #d = debug_packet(rs)

                        #self.debug_delegate.dispatch(d)

                    elif pkt_code == 2:

                        #Jiajun: pass quaterian through serial

                        #print ('code2')
                        
                        p = quat_packet(rs)

                        #Jiajun
                        #g_quat[0] = p.q0
                        #g_quat[1] = p.q1
                        #g_quat[2] = p.q2
                        #g_quat[3] = p.q3

                        #p.display_raw()
                        
                        #sss = '%f, %f, %f, %f' %\
                            #(p.q0,p.q1,p.q2,p.q3)
                        #print(sss)

                        global COMPORT1
                        
                        if self.com == COMPORT1:
                            self.quat_delegate.dispatch(p,0) 
                        else:
                            self.quat_delegate.dispatch(p,1)

                    elif pkt_code == 3:

                        #Jiajun: pass accelerator through serial
                        #print ('code3')

                        d = data_packet(rs)
                        #d.display_raw()
                        
                        self.data_delegate.dispatch(d)

                    else:
                        sss = 'no handler for pkt_code',pkt_code

                else:

                    c = ' '
                    sss = 'serial misaligned!'
                    print (sss)

                    while not ord(c) == ord('$'):

                        c = self.s.read(1)

                    self.s.read(NUM_BYTES-1)



        def write(self,a):

            self.s.write(a)


        def close(self):

            self.s.close()



        def write_log(self,fname):

            f = open(fname,'w')
            

            for p in self.packets:

                f.write(p.logfile_line())

            f.close()
            
# For 16-bit signed integers.
def two_bytes(d1,d2):
        d = ord(d1)*256 + ord(d2)
        if d > 32767:
            d -= 65536

        return d

def two_bytes_no_check(d1, d2):
        d = ord(d1)*256 + ord(d2)
        return d        

# For 32-bit signed integers.            
def four_bytes(d1, d2, d3, d4):
    d = ord(d1)*(1<<24) + ord(d2)*(1<<16) + ord(d3)*(1<<8) + ord(d4)
    if d > 2147483648:
        d-= 4294967296

    return d
   
def draw_cross(img, x, y, color=(0,255,0)):
    xi = int(x)
    yi = int(y)
    cv2.line(img, (xi-5,yi), (xi+5,yi), color, 1)
    cv2.line(img, (xi,yi-5), (xi,yi+5), color, 1)
    
def align_array_by_min(array):
    s = len(array)
    min_value = array[0]
    min_index = 0
    for i in range(0,s):
        if array[i]<min_value:
            min_value = array[i]
            min_index = i
    new_array = [array[(i+min_index)%s] for i in range(0,s)]
    return new_array

def angle_two_vector(sx,sy,nx,ny, direction = False):
    s_mod = math.sqrt(sx*sx+sy*sy)
    if s_mod<0.000001:
        return 0
    sx /= s_mod
    sy /= s_mod
    n_mod = math.sqrt(nx*nx+ny*ny)
    if n_mod<0.000001:
        return 0
    nx /= n_mod
    ny /= n_mod
    angle = math.acos(sx*nx+sy*ny)/math.pi*180.0
    if not direction:
        if angle>90:
            angle = 180-angle
    return angle
    
    
def select_sort_value(value, ascend = True):
    s = len(value)
    flag = [0 for i in range(0,s)]
    new_value = [0 for i in range(0,s)]
    
    ptr = 0
    while 1:
        max_value = 0
        max_index = -1
        for i in range(0,s):
            if flag[i] == 0:
                if max_index < 0:
                    max_value = value[i]
                    max_index = i                    
                elif value[i]>max_value:
                    max_value = value[i]
                    max_index = i
                    
        if max_index<0:
            break
        
        new_value[ptr] = max_value
        flag[max_index] = 1
        ptr = ptr+1
        
        if ptr == s:
            break;
        
    if ascend:
        resort_value = [0 for i in range(0,s)]
        for i in range(0,s):
            resort_value[i] = new_value[s-1-i]        
         
        return resort_value
    
    return new_value
           
def select_sort(index, value, ascend = True):
    s = len(index)
    flag = [0 for i in range(0,s)]
    new_index = [0 for i in range(0,s)]
    
    ptr = 0
    while 1:
        max_value = 0
        max_index = -1
        for i in range(0,s):
            if flag[i] == 0:
                if max_index < 0:
                    max_value = value[i]
                    max_index = i                    
                elif value[i]>max_value:
                    max_value = value[i]
                    max_index = i
                    
        if max_index<0:
            break
        
        new_index[ptr] = max_index
        flag[max_index] = 1
        ptr = ptr+1
        
        if ptr == s:
            break;
        
    if ascend:
        resort_index = [0 for i in range(0,s)]
        for i in range(0,s):
            resort_index[i] = new_index[s-1-i]        
         
        return resort_index
    
    return new_index
        

class data_packet (object):

        def __init__(self, l):
            global curr_light;

            self.l = l
            
            self.data = [0,0,0,0,0,0,0,0,0]

            self.type = ord(l[2])
            
            #print(self.type)

            if self.type == 0:   # accel

                self.data[0] = four_bytes(l[3],l[4],l[5],l[6]) * 1.0 / (1<<16)

                self.data[1] = four_bytes(l[7],l[8],l[9],l[10]) * 1.0 / (1<<16)

                self.data[2] = four_bytes(l[11],l[12],l[13],l[14]) * 1.0 / (1<<16)
                
                global acc_x,acc_y,acc_z
                acc_x = self.data[0]
                acc_y = self.data[1]
                acc_z = self.data[2]

            elif self.type == 1:   # gyro

                self.data[0] = four_bytes(l[3],l[4],l[5],l[6]) * 1.0 / (1<<16)

                self.data[1] = four_bytes(l[7],l[8],l[9],l[10]) * 1.0 / (1<<16)

                self.data[2] = four_bytes(l[11],l[12],l[13],l[14]) * 1.0 / (1<<16)
              

            elif self.type == 2:   # compass

                self.data[0] = four_bytes(l[3],l[4],l[5],l[6]) * 1.0 / (1<<16)

                self.data[1] = four_bytes(l[7],l[8],l[9],l[10]) * 1.0 / (1<<16)

                self.data[2] = four_bytes(l[11],l[12],l[13],l[14]) * 1.0 / (1<<16)
             

            elif self.type == 3:   # quat

                self.data[0] = four_bytes(l[3],l[4],l[5],l[6]) * 1.0 / (1<<30)

                self.data[1] = four_bytes(l[7],l[8],l[9],l[10]) * 1.0 / (1<<30)

                self.data[2] = four_bytes(l[11],l[12],l[13],l[14]) * 1.0 / (1<<30)

                self.data[3] = four_bytes(l[15],l[16],l[17],l[18]) * 1.0 / (1<<30)

            elif self.type == 4:   # euler

                self.data[0] = four_bytes(l[3],l[4],l[5],l[6]) * 1.0 / (1<<16)

                self.data[1] = four_bytes(l[7],l[8],l[9],l[10]) * 1.0 / (1<<16)

                self.data[2] = four_bytes(l[11],l[12],l[13],l[14]) * 1.0 / (1<<16)
                

            elif self.type == 5:   # rot

                self.data[0] = two_bytes(l[3],l[4]) * 1.0 / (1<<14)

                self.data[1] = two_bytes(l[5],l[6]) * 1.0 / (1<<14)

                self.data[2] = two_bytes(l[7],l[8]) * 1.0 / (1<<14)

                self.data[3] = two_bytes(l[9],l[10]) * 1.0 / (1<<14)

                self.data[4] = two_bytes(l[11],l[12]) * 1.0 / (1<<14)

                self.data[5] = two_bytes(l[13],l[14]) * 1.0 / (1<<14)

                self.data[6] = two_bytes(l[15],l[16]) * 1.0 / (1<<14)

                self.data[7] = two_bytes(l[17],l[18]) * 1.0 / (1<<14)

                self.data[8] = two_bytes(l[19],l[20]) * 1.0 / (1<<14)
                

            elif self.type == 6:   # heading

                self.data[0] = four_bytes(l[3],l[4],l[5],l[6]) * 1.0 / (1<<16)

            elif self.type == 7:
                self.data[0] = two_bytes_no_check(l[3],l[4])  # Pressure
                self.data[0] =  self.data[0] * 60 / 65535 + 50
                self.data[0] =  self.data[0] * 10
                
                self.data[1] =  two_bytes_no_check(l[5],l[6]) # Humidity
                s= struct.pack('f', self.data[1])
                i= struct.unpack('i',s)
                temp = struct.pack('f', 65528)
                i_temp= struct.unpack('i',temp)
                tm = ( i_temp[0] & i[0])
                temp = struct.pack('i', tm)
                i_temp= struct.unpack('f',temp)
                self.data[1] =i_temp[0]                
                self.data[1] = (float) (-6 + 125 * ( self.data[1]/ (2**16)));
                
                self.data[2] =  two_bytes_no_check(l[7],l[8]) # Temperature
                self.data[2] = (float) (-46.85 + 175.72 * (self.data[2]/ (2.0**16.0)))
                        
                self.data[3] = two_bytes_no_check(l[9],l[10])# Light
                self.data[3] = self.data[3] * (float) (0.06103) # for CM323                        
                curr_light = (self.data[3] * (float) (0.8)) + (curr_light * (float)(0.2))
                curr_light = round(curr_light)
                self.data[3] = curr_light;
                
                uv= two_bytes_no_check(l[11],l[12])
                self.data[4] = (uv * (float)(0.022)) * 10; #UV
                array_es[0]=self.data[0]
                array_es[1]=self.data[1]
                array_es[2]=self.data[2]
                array_es[3]=self.data[3]
                array_es[4]=self.data[4]
            elif self.type == 8:
                self.data[0] = four_bytes(l[3],l[4],l[5],l[6]) * 1.0 / (1<<16)
                self.data[1] = four_bytes(l[7],l[8],l[9],l[10]) * 1.0 / (1<<16)
                self.data[2] = four_bytes(l[11],l[12],l[13],l[14]) * 1.0 / (1<<16)
                self.data[3] = four_bytes(l[15],l[16],l[17],l[18]) * 1.0
                
                global c_vel_x,c_vel_y,c_vel_z
                c_vel_x = self.data[0]
                c_vel_y = self.data[1]
                c_vel_z = self.data[2]
                
            else:   # unsupported
                pass

        def display_raw(self):
                l = self.l
                sss = ''.join(
                            [ str(ord(l[0])), ' '] + \

                            [ str(ord(l[1])), ' '] + \

                            [ str(ord(a)).ljust(4) for a in 

                                                                    [ l[2], l[3], l[4], l[5], l[6], l[7], l[8], l[9], l[10] ] ] + \

                            [ str(ord(a)).ljust(4) for a in 

                                                                    [ l[11], l[12], l[13], l[14], l[15], l[16], l[17], l[18], l[19], l[20]] ]

                )
                print (sss)
        def display(self):
            print('display data')
            
            global f_file
            global es_file
            global c_file
            global count
            global count_type
            global l
            global log
            global set_total_1
            global array_a
            global array_g
            global array_c
            global array_q
            global array_e
            global array_rm
            global array_h
            global array_es
            global log_accel
            global log_gyro
            global log_compass
            global log_rotmatrix
            global log_eular
            global log_quat
            global log_external_sensors
            global log_heading

            global accel_d_enable
            global gyro_d_enable
            global compass_d_enable
            global euler_d_enable
            global rotmat_d_enable
            global heading_d_enable
            global quat_d_enable
            global exter_sensor_d_enable            


            millis = int(round(time.time() * 1000))
            
            if self.type == 0:
                    accel_d_enable= "Enable"
                    sss = 'accel, %7.3f, %7.3f, %7.3f' % \
                          (self.data[0], self.data[1], self.data[2])
                    print (sss)
                
                    count=count +1
                    count_type= count_type+1;
                    array_a[0]=self.data[0]
                    array_a[1]=self.data[1]
                    array_a[2]=self.data[2] 

            elif self.type == 1:
                    gyro_d_enable= "Enable"
                    sss = 'gyro: %9.5f %9.5f %9.5f' % \
                          (self.data[0], self.data[1], self.data[2])
                    print (sss)
                    count=count +1
                    array_g[0]=self.data[0]
                    array_g[1]=self.data[1]
                    array_g[2]=self.data[2] 
                    
            elif self.type == 2:
                    compass_d_enable= "Enable"
                    sss = 'compass: %7.4f %7.4f %7.4f' % \
                          (self.data[0], self.data[1], self.data[2])
                    print (sss)
                    count=count +1
                    array_c[0]=self.data[0]
                    array_c[1]=self.data[1]
                    array_c[2]=self.data[2] 
                    
            elif self.type == 3:
                    quat_d_enable= "Enable"
                    sss = 'quat: %7.4f %7.4f %7.4f %7.4f' % \
                          (self.data[0], self.data[1], self.data[2], self.data[3])
                    print (sss)
                    count=count +1
                    array_q[0]=self.data[0]
                    array_q[1]=self.data[1]
                    array_q[2]=self.data[2]
                    array_q[3]=self.data[3] 
                    
            elif self.type == 4:
                    euler_d_enable= "Enable"
                    sss = 'euler: %7.4f %7.4f %7.4f' % \
                          (self.data[0], self.data[1], self.data[2])
                    print (sss)
                    count=count +1
                    array_e[0]=self.data[0]
                    array_e[1]=self.data[1]
                    array_e[2]=self.data[2] 


            elif self.type == 5:
                    rotmat_d_enable= "Enable"
                    sss = 'rotation matrix: \n%7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f' % \
                          (self.data[0], self.data[1], self.data[2], self.data[3], \
                           self.data[4], self.data[5], self.data[6], self.data[7], \
                           self.data[8])
                    print (sss)
                    count=count +1
                    array_rm[0]=self.data[0]
                    array_rm[1]=self.data[1]
                    array_rm[2]=self.data[2]
                    array_rm[3]=self.data[3]
                    array_rm[4]=self.data[4]
                    array_rm[5]=self.data[5]
                    array_rm[6]=self.data[6]
                    array_rm[7]=self.data[7]
                    array_rm[8]=self.data[8]
                    

            elif self.type == 6:
                    heading_d_enable= "Enable"
                    sss = 'heading: %7.4f' % self.data[0]
                    print (sss)
                    count=count +1
                    array_h[0]=self.data[0]
                    
                
            elif self.type == 7:
                    exter_sensor_d_enable= "Enable"
                    print ('Sensors (Pressure, Humidity, Temperature, Light, UV):\n %.2f %.2f %.2f %.2f %.2f' % \
                           (array_es[0], array_es[1], array_es[2],array_es[3],array_es[4]))
                            #(self.data[0], self.data[1], self.data[2],self.data[3],self.data[4]))
                    count=count +1
                    
                    #if (log_es ==True):
                        #if(log_external_sensors  == True): #UV, TEMP, Pressure, Light, humidity in one file
                            #es_file.writerow(list([millis,array_es[0],array_es[1],array_es[2],array_es[3],array_es[4]]))
                            
            elif self.type == 8:
                    #print('pos')
                    pass
                    
            else:
                print ('what?')
                
            if (log ==True):                
       
                if(count >= set_total_1 and set_total_1>0):
                    count=0
                    l.extend([millis])
                    if(log_accel ==True):
                        l.extend([array_a[0],array_a[1],array_a[2]])

                    if(log_gyro ==True):
                        l.extend([array_g[0],array_g[1],array_g[2]])

                    if(log_compass ==True):
                        l.extend([array_c[0],array_c[1],array_c[2]])

                    if(log_eular ==True):
                        l.extend([array_e[0],array_e[1],array_e[2]])

                    if(log_quat ==True):
                        l.extend([array_q[0],array_q[1],array_q[2],array_q[3]])
                        

                    if(log_rotmatrix ==True):
                        l.extend([array_rm[0],array_rm[1],array_rm[2],array_rm[3],array_rm[4],array_rm[5],array_rm[6],array_rm[7],array_rm[8]])
                        

                    if(log_heading ==True):
                        l.extend([array_h[0]])
                    f_file.writerow(list(l))
                    del l[:]

      
          
class quat_packet (object):

        def __init__(self, l):
            self.l = l
            
            #sss = ''.join([str(ord(l[0])),' ']+[str(ord(l[1])),' ']+[str(ord(a)).ljust(4) for a in [l[2],l[3],l[4],l[5],l[6],l[7],l[8],l[9],l[10]]]+[ str(ord(a)).ljust(4) for a in [l[8],l[9],l[10],l[11],l[12],l[13]]])
            #print (sss)

            self.q0 = four_bytes(l[3],l[4],l[5],l[6]) * 1.0 / (1<<30)

            self.q1 = four_bytes(l[7],l[8],l[9],l[10]) * 1.0 / (1<<30)

            self.q2 = four_bytes(l[11],l[12],l[13],l[14]) * 1.0 / (1<<30)

            self.q3 = four_bytes(l[15],l[16],l[17],l[18]) * 1.0 / (1<<30)
            
        def display_raw(self):
                l = self.l
                sss = ''.join(
                            [ str(ord(l[0])), ' '] + \

                            [ str(ord(l[1])), ' '] + \

                            [ str(ord(a)).ljust(4) for a in 

                                                                    [ l[2], l[3], l[4], l[5], l[6], l[7], l[8], l[9], l[10] ] ] + \

                            [ str(ord(a)).ljust(4) for a in 

                                                                    [ l[8], l[9], l[10] , l[11], l[12], l[13]] ]

                )
                print (sss)


        def display(self):

            if 1:
                    sss = 'qs ' + ' '.join([str(s).ljust(15) for s in
                                            [ self.q0, self.q1, self.q2, self.q3 ]])
                    print (sss)


            if 0:

                euler0, euler1, euler2 = self.to_q().get_euler()
                sss = 'eulers ' + ' '.join([str(s).ljust(15) for s in
                                            [ euler0, euler1, euler2 ]])

                print (sss)
                            
            if 0:

                euler0, euler1, euler2 = self.to_q().get_euler()
                sss = 'eulers ' + ' '.join([str(s).ljust(15) for s in
                                            [ (euler0 * 180.0 / 3.14159) - 90 ]])
                print (sss)



        def to_q(self):

            return Quaternion(self.q0, self.q1, self.q2, self.q3)
        
class packet_delegate(object):
        def loop(self,event):
            sss = 'generic packet_delegate loop w/event',event
            print (sss)
                    
        def dispatch(self,p):
            sss = 'generic packet_delegate dispatched',p
            print (sss)

class empty_packet_delegate(packet_delegate):
        def loop(self,event):
            pass

        def dispatch(self,p):
            print('empty packet dispatch')
            pass            
        
class cube_viewer (packet_delegate):
    def __init__(self):
        self.screen = Screen(500,500,scale=0.5)
        self.cube1 = Cube(30,60,10,200)
        self.cube2 = Cube(60,120,10)
        self.q = Quaternion(1,0,0,0)
        self.previous = None  # previous quaternion
        self.latest1 = None   
        self.latest2 = None
        
    def loop(self,event):
        packet1 = self.latest1
        packet2 = self.latest2
        if packet1:
            #print('draw')
            q = packet1.to_q().normalized()
            global QUAT_DET
            QUAT_DET = q
            q = QUAT_DET.__mul__(QUAT_TRANS)            
            self.cube1.erase2D(self.screen)
            self.cube1.draw2D(self.screen,q, 320-DELTA_X, 240-DELTA_Y)            
            pygame.display.flip()
            self.latest1 = None
        if packet2:
            q = packet2.to_q().normalized()
            global QUAT_TUB
            QUAT_TUB = q
            #q = QUAT_TUB.__mul__(QUAT_TRANS)            	
            self.cube2.erase2D(self.screen)
            self.cube2.draw2D(self.screen, q, 0, 0)
            pygame.display.flip()
            self.latest1 = None
        #else:
            #q = Quaternion(1,0,0,0)
            #self.cube1.erase(self.screen)
            #self.cube1.draw2D(self.screen, q, 100, 0)
            #self.cube2.erase(self.screen)
            #self.cube2.draw2D(self.screen, q, -100, 0)
            #pygame.display.flip()

    def dispatch(self,p,i=0):
        #print('cube_viewer dispatch')
        if isinstance(p,quat_packet):
            if i:
                self.latest2 = p
            else:
                self.latest1 = p
    
    def close(self):
        pygame

class data_viewer (packet_delegate):
        def loop(self,event):
            pass

        def dispatch(self,p):
            print('data viewer dispatch')
            assert isinstance(p,data_packet);
            p.display()
                        
# main function
if __name__ == '__main__':

#configuration file    
    config = ConfigParser.SafeConfigParser()
    config.read('c:\\tmp\\detector_surface.ini')
    
    if CAMERA_ON:    
        if config.getboolean('CameraCalibration', 'Calibrated'):
            print('Camera calibration loaded')
            fx = config.getfloat('CameraCalibration', 'fx')
            fy = config.getfloat('CameraCalibration', 'fy')
            cx = config.getfloat('CameraCalibration', 'cx')
            cy = config.getfloat('CameraCalibration', 'cy')
            k1 = config.getfloat('CameraCalibration', 'k1')
            k2 = config.getfloat('CameraCalibration', 'k2')
            k3 = config.getfloat('CameraCalibration', 'k3')
            p1 = config.getfloat('CameraCalibration', 'p1')
            p2 = config.getfloat('CameraCalibration', 'p2')
        else:
            print('Camera calibration required')
            sys.exit(0) 
    
    #     calculate camera distortion matrix    
        mtx = np.array([(fx, 0.0, cx), (0.0, fy, cy), (0.0, 0.0, 1.0)])
        dist = np.array([k1,k2,p1,p2,k3])   
        print(mtx)
        print(dist)
        width = 640
        height = 480
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (width, height), 1, (width, height))
        mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (width, height), 5)
        #FOCAL_LENGTH = (fx+fy)/2
        
    if GYRO_ON:        
    #     load pose calibration results between two gyro sensors
        if config.getboolean('GyroCalibration', 'Calibrated'):
            print('Gyro calibration loaded')
            QUAT_TRANS.w = config.getfloat('GyroCalibration','w')
            QUAT_TRANS.x = config.getfloat('GyroCalibration','x')
            QUAT_TRANS.y = config.getfloat('GyroCalibration','y')
            QUAT_TRANS.z = config.getfloat('GyroCalibration','z')
            QUAT_TRANS.normalize()
            print(QUAT_TRANS)
        else:
            print('Gryo calibration required')
            cal_viewer = cube_viewer()
            cal_data = data_viewer()
            
            cal_reader1 = packet_reader(COMPORT1, cal_viewer, cal_data)
            sleep(2)
            cal_reader2 = packet_reader(COMPORT2, cal_viewer, cal_data)
            while 1:
                event = pygame.event.poll()
                cal_reader1.read()
                cal_reader2.read()
                cal_viewer.loop(event)        	
                pygame.time.delay(0)
    
                if QUAT_DET and QUAT_TUB :
                    trans = QUAT_DET.conjugated().__mul__(QUAT_TUB).normalize()
                    if config.has_section('GyroCalibration'):
                        print('update gyro calibration results')                	
                        config.set('GyroCalibration', 'w', '%f'%trans.w)
                        config.set('GyroCalibration', 'x', '%f'%trans.x)
                        config.set('GyroCalibration', 'y', '%f'%trans.y)
                        config.set('GyroCalibration', 'z', '%f'%trans.z)
                        config.set('GyroCalibration', 'calibrated', 'true')
                        print(QUAT_DET)
                        print(QUAT_TUB)
                        print(trans)
                        with open('c:\\tmp\\detector_surface.ini','w') as configfile:
                            config.write(configfile)
                    
                    cal_reader1.close()
                    cal_reader2.close()
                        
                    sys.exit(0)
        
    
    pygame.init()
    viewer = cube_viewer() 
    data = data_viewer()
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    if GYRO_ON:      
        reader1 = packet_reader(COMPORT1, viewer, data)
        sleep(2)
        reader2 = packet_reader(COMPORT2, viewer, data)
    
    if CAMERA_ON:
        cap = cv2.VideoCapture(0)          
    
    while CAMERA_ON or GYRO_ON:
        dist_det2tub = -1
        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            viewer.close() 
            break
        
        if GYRO_ON: 
            reader1.read()
            reader2.read()         
            viewer.loop(event)         
            data.loop(event)         
            pygame.time.delay(0)
        
#             print('loop')
#             print(QUAT_DET)
#             print(QUAT_TUB)
        
        if CAMERA_ON:
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            #gray = cv2.flip(gray,0)
            #cv2.imshow('original',gray)
            img = cv2.remap(gray, mapx, mapy, cv2.INTER_LINEAR)
            frame = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            
            if GYRO_ON:
                ir = ir_marker()
                q1 = QUAT_TUB
                q2 = QUAT_DET.__mul__(QUAT_TRANS)
                qt = q2.conjugated().__mul__(q1).normalize()
                ir.rotate(qt)
                ir_pts = ir.perspective_transform()
                #draw rot axis
                rot_axis = ir.get_perspective_axis()
                ox = 240
                oy = -160
                cv2.line(frame, (IMAGE_WIDTH/2+ox,IMAGE_HEIGHT/2+oy),(int(rot_axis[0].x)+ox,int(rot_axis[0].y)+oy),(0,0,255))#x, red
                cv2.line(frame, (IMAGE_WIDTH/2+ox,IMAGE_HEIGHT/2+oy),(int(rot_axis[1].x)+ox,int(rot_axis[1].y)+oy),(0,255,0))#y, green
                cv2.line(frame, (IMAGE_WIDTH/2+ox,IMAGE_HEIGHT/2+oy),(int(rot_axis[2].x)+ox,int(rot_axis[2].y)+oy),(255,0,0))#z, blue
                
                #ir_pts = ir.flip(ir_pts, 0)
                for i in range(0, len(ir_pts)):
                    draw_cross(frame, ir_pts[i].x, ir_pts[i].y, (0,0,255))
                    #cv2.putText(frame,'%d'%i, (int(ir_pts[i].x)+10,int(ir_pts[i].y)+10), font, 0.5, (255,255,255), 1, cv2.LINE_AA)
                    
#                 for i in range(0,3):
#                     cv2.line(frame,(int(ir_pts[4+i].x),int(ir_pts[4+i].y)),(int(ir_pts[5+i].x),int(ir_pts[5+i].y)),(0,0,255),1)
#                 cv2.line(frame,(int(ir_pts[4].x),int(ir_pts[4].y)),(int(ir_pts[7].x),int(ir_pts[7].y)),(0,0,255),1)
            
            ret, thres_img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
            #ret, markers = cv2.connectedComponents(thres_img)
            _, contours, hierarchy = cv2.findContours(thres_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            con_num = len(contours)
            #print(con_num)
            if con_num == 4:
                x=[0,0,0,0]
                y=[0,0,0,0]
                sx = 0
                sy = 0
                ptr = 0
                for cnt in contours:
                    (cx,cy), radius = cv2.minEnclosingCircle(cnt)
                    sx += cx
                    sy += cy
                    x[ptr] = cx
                    y[ptr] = cy
                    #draw_cross(frame, cx, cy)
                    cv2.circle(frame,(int(cx),int(cy)),6,(0,255,0))
                    ptr = ptr+1
                DELTA_X = sx/4
                DELTA_Y = sy/4
                draw_cross(frame, DELTA_X, DELTA_Y, (0,255,255))
                
                #resort tracking points
                angles = [0,0,0,0]
                for i in range(0,4):
                    dx = x[i]-DELTA_X
                    dy = y[i]-DELTA_Y
                    angles[i] = math.atan2(dy, dx)
                    if angles[i]<0:
                        angles[i] = angles[i]+2*math.pi
                        
                index = [i for i in range(0,4)]
                new_index = select_sort(index, angles)
                nx=[0,0,0,0]
                ny=[0,0,0,0]
                for i in range(0,4):
                    nx[i] = x[new_index[i]]
                    ny[i] = y[new_index[i]]
                    #cv2.putText(frame,'%d'%i, (int(nx[i])+10,int(ny[i])+10), font, 0.5, (255,255,255), 1, cv2.LINE_AA)
                    cv2.circle(frame,(int(ir_pts[i].x),int(ir_pts[i].y)),6,(0,255,255))
#                 vertex_x = [0,0,0,0]
#                 vertex_y = [0,0,0,0]
#                 for i in range(0,4):
#                     dnx = nx[i]-IMAGE_WIDTH/2
#                     dny = ny[i]-IMAGE_HEIGHT/2
#                     dn_mod = math.sqrt(dnx*dnx+dny*dny)
#                     dnx/=dn_mod
#                     dny/=dn_mod
#                     vertex_x[i] = dnx*dn_mod*1.05+IMAGE_WIDTH/2
#                     vertex_y[i] = dny*dn_mod*1.05+IMAGE_HEIGHT/2
#                 for i in range(0,4):
#                     cv2.line(frame,(int(vertex_x[i]),int(vertex_y[i])),(int(vertex_x[(i+1)%4]),int(vertex_y[(i+1)%4])),(0,255,0),1)
                
                if GYRO_ON:
                    dist_pts = math.sqrt((ir_pts[0].x-ir_pts[2].x)*(ir_pts[0].x-ir_pts[2].x)+(ir_pts[0].y-ir_pts[2].y)*(ir_pts[0].y-ir_pts[2].y)) 
                    dist_pts = dist_pts + math.sqrt((ir_pts[1].x-ir_pts[3].x)*(ir_pts[1].x-ir_pts[3].x)+(ir_pts[1].y-ir_pts[3].y)*(ir_pts[1].y-ir_pts[3].y))
                    dist_img = math.sqrt((nx[0]-nx[2])*(nx[0]-nx[2])+(ny[0]-ny[2])*(ny[0]-ny[2]))
                    dist_img = dist_img + math.sqrt((nx[1]-nx[3])*(nx[1]-nx[3])+(ny[1]-ny[3])*(ny[1]-ny[3]))
#                     angles = [0,0,0,0]
#                     cenx = 0
#                     ceny = 0
#                     for i in range(0,4):
#                         cenx += ir_pts[i].x
#                         ceny += ir_pts[i].y
#                     for i in range(0,4):
#                         dx = ir_pts[i].x-cenx
#                         dy = ir_pts[i].y-ceny
#                         angles[i] = math.atan2(dy, dx)
#                         if angles[i]<0:
#                             angles[i] = angles[i]+2*math.pi
#                     pts_index = select_sort([i for i in range(0,4)], angles)
#                     vertex_x = [0,0,0,0]
#                     vertex_y = [0,0,0,0]
#                     for i in range(0,4):
#                         px = ir_pts[pts_index[i]].x-cenx
#                         py = ir_pts[pts_index[i]].y-ceny
#                         dist1 = math.sqrt(px*px+py*py)
#                         px = ir_pts[pts_index[i]+4].x-cenx
#                         py = ir_pts[pts_index[i]+4].y-ceny
#                         dist_ratio = math.sqrt(px*px+py*py)/dist1
#                          
#                         dnx = nx[i]-DELTA_X
#                         dny = ny[i]-DELTA_Y
#                         dn_mod = math.sqrt(dnx*dnx+dny*dny)
#                         dnx/=dn_mod
#                         dny/=dn_mod
#                          
#                         vertex_x[i] = dnx*dn_mod*dist_ratio+DELTA_X
#                         vertex_y[i] = dny*dn_mod*dist_ratio+DELTA_Y
#                     for i in range(0,4):
#                         cv2.line(frame,(int(vertex_x[i]),int(vertex_y[i])),(int(vertex_x[(i+1)%4]),int(vertex_y[(i+1)%4])),(0,255,0),1)
                    
                else:
                    #roughly decide distance between detector and tube
                    maker = ir_marker()
                    maker_pts = maker.perspective_transform()
                    dist_pts = math.sqrt((maker_pts[0].x-maker_pts[2].x)*(maker_pts[0].x-maker_pts[2].x)+(maker_pts[0].y-maker_pts[2].y)*(maker_pts[0].y-maker_pts[2].y)) 
                    dist_pts = dist_pts + math.sqrt((maker_pts[1].x-maker_pts[3].x)*(maker_pts[1].x-maker_pts[3].x)+(maker_pts[1].y-maker_pts[3].y)*(maker_pts[1].y-maker_pts[3].y))
                    dist_img = math.sqrt((nx[0]-nx[2])*(nx[0]-nx[2])+(ny[0]-ny[2])*(ny[0]-ny[2]))
                    dist_img = dist_img + math.sqrt((nx[1]-nx[3])*(nx[1]-nx[3])+(ny[1]-ny[3])*(ny[1]-ny[3]))

                dist_det2tub = dist_pts/dist_img*FOCAL_LENGTH 
                    
                #print(DELTA_X, DELTA_Y)
            elif con_num<2:
                pass
            elif con_num==3:
                x=[0,0,0]
                y=[0,0,0]
                ptr = 0
                for cnt in contours:
                    (cx,cy), radius = cv2.minEnclosingCircle(cnt)
                    #draw_cross(frame, cx, cy)
                    cv2.circle(frame,(int(cx),int(cy)),6,(0,255,0))
                    x[ptr] = cx
                    y[ptr] = cy
                    ptr = ptr + 1
                cen_x = 0
                cen_y = 0
                dist = [0,0,0]
                for i in range(0,3):
                    sx = x[i]
                    sy = y[i]
                    ex = x[(i+1)%3]
                    ey = y[(i+1)%3]
                    dist[i] = math.sqrt((sx-ex)*(sx-ex)+(sy-ey)*(sy-ey))
                max_dist = dist[0]
                max_index = 0
                for i in range(1,3):
                    if dist[i]>max_dist:
                        max_dist = dist[i]
                        max_index = i  
                DELTA_X = (x[max_index]+x[(max_index+1)%3])/2
                DELTA_Y = (y[max_index]+y[(max_index+1)%3])/2
                draw_cross(frame, DELTA_X, DELTA_Y, (0,255,255))
                dist_img = max_dist
                
                #resort tracking points
                angles = [0,0,0]
                for i in range(0,3):
                    dx = x[i]-DELTA_X
                    dy = y[i]-DELTA_Y
                    angles[i] = math.atan2(dy, dx)
                    if angles[i]<0:
                        angles[i] = angles[i]+2*math.pi
                        
                index = [i for i in range(0,3)]
                new_index = select_sort(index, angles)
                nx=[0,0,0]
                ny=[0,0,0]
                for i in range(0,3):
                    nx[i] = x[new_index[i]]
                    ny[i] = y[new_index[i]]
                    #cv2.putText(frame,'%d'%i, (int(nx[i])+10,int(ny[i])+10), font, 0.5, (255,255,255), 1, cv2.LINE_AA)
                        
                if GYRO_ON:
                    cenx = (nx[0]+nx[1]+nx[2])/3
                    ceny = (ny[0]+ny[1]+ny[2])/3
                    img_dx = cenx-DELTA_X
                    img_dy = ceny-DELTA_Y
                    pnt_indices = [[0,1,2],[0,1,3],[0,2,3],[1,2,3]]
                    angle_diff = [0,0,0,0]
                    #print('image pts',img_dx,img_dy)
                    for i in range(0,4):
                        com_pts = [ir_pts[pnt_indices[i][0]],ir_pts[pnt_indices[i][1]],ir_pts[pnt_indices[i][2]]]
                        cenx = (com_pts[0].x+com_pts[1].x+com_pts[2].x)/3
                        ceny = (com_pts[0].y+com_pts[1].y+com_pts[2].y)/3
                        dx = cenx-IMAGE_WIDTH/2
                        dy = ceny-IMAGE_HEIGHT/2
                        angle_diff[i] = angle_two_vector(img_dx, img_dy, dx, dy, True)
                    #print(angle_diff)
                    min_angle_diff = angle_diff[0]
                    min_angle_index = 0
                    for i in range(1,4):
                        if angle_diff[i]<min_angle_diff:
                            min_angle_diff = angle_diff[i]
                            min_angle_index = i
                    
                    circle_index = [pnt_indices[min_angle_index][0],pnt_indices[min_angle_index][1],pnt_indices[min_angle_index][2]]
                    for i in range(0,3):
                        cv2.circle(frame,(int(ir_pts[circle_index[i]].x),int(ir_pts[circle_index[i]].y)),6,(0,255,255))
                    #calculate distance in space
                    p1 = 0
                    p2 = 0         
                    if min_angle_index==0 or min_angle_index==3:
                        p1 = pnt_indices[min_angle_index][0]
                        p2 = pnt_indices[min_angle_index][2]
                    elif min_angle_index==1:
                        p1 = pnt_indices[min_angle_index][1]
                        p2 = pnt_indices[min_angle_index][2]
                    else:
                        p1 = pnt_indices[min_angle_index][0]
                        p2 = pnt_indices[min_angle_index][1]
                    dist_pts = math.sqrt((ir_pts[p1].x-ir_pts[p2].x)*(ir_pts[p1].x-ir_pts[p2].x)+(ir_pts[p1].y-ir_pts[p2].y)*(ir_pts[p1].y-ir_pts[p2].y))
                    #draw detector
#                     cenx = (ir_pts[p1].x+ir_pts[p2].x)/2
#                     ceny = (ir_pts[p1].y+ir_pts[p2].y)/2
#                     index = pnt_indices[min_angle_index]
#                     angles = [0,0,0]
#                     for i in range(0,3):
#                         dx = ir_pts[index[i]].x-cenx
#                         dy = ir_pts[index[i]].y-ceny
#                         angles[i] = math.atan2(dy, dx)
#                         if angles[i]<0:
#                             angles[i] += math.pi*2
#                     new_index = select_sort(index, angles)
#                     vertex_x = [0,0,0]
#                     vertex_y = [0,0,0]
#                     ratio = 0
#                     for i in range(0,3):
#                         px = ir_pts[new_index[i]].x-cenx
#                         py = ir_pts[new_index[i]].y-ceny
#                         dist1 = math.sqrt(px*px+py*py)
#                         px = ir_pts[new_index[i]+4].x-cenx
#                         py = ir_pts[new_index[i]+4].y-ceny
#                         dist_ratio = math.sqrt(px*px+py*py)/dist1
#                          
#                         dnx = nx[i]-DELTA_X
#                         dny = ny[i]-DELTA_Y
#                         dn_mod = math.sqrt(dnx*dnx+dny*dny)
#                         dnx/=dn_mod
#                         dny/=dn_mod
#                          
#                         vertex_x[i] = dnx*dn_mod*dist_ratio+DELTA_X
#                         vertex_y[i] = dny*dn_mod*dist_ratio+DELTA_Y
#                         
#                         ratio += (nx[i]-DELTA_X)/(ir_pts[new_index[i]].x-cenx)
#                     ratio /= 3
#                     p4 = 0
#                     if min_angle_index==0:
#                         p4 = 3
#                     elif min_angle_index==1:
#                         p4 = 2
#                     elif min_angle_index==2:
#                         p4 = 1
#                     else:
#                         p4 = 0
#                     dx = ir_pts[p4+4].x-cenx
#                     dy = ir_pts[p4+4].y-ceny
#                     d_mod = math.sqrt(dx*dx+dy*dy)
#                     dx/=d_mod
#                     dy/=d_mod
#                     d_mod *= ratio
#                     nx.append(dx*d_mod+DELTA_X)
#                     ny.append(dy*d_mod+DELTA_Y)
#                     angles = [0,0,0,0]
#                     for i in range(0,4):
#                         dx = nx[i]-DELTA_X
#                         dy = ny[i]-DELTA_Y
#                         angles[i] = math.atan2(dy,dx)
#                         if angles[i]<0:
#                             angles[i]+=math.pi*2
#                     n_index = select_sort([i for i in range(0,4)], angles)
#                     for i in range(0,4):                   
#                         cv2.line(frame,(int(nx[n_index[i]]),int(ny[n_index[i]])),(int(nx[n_index[(i+1)%4]]),int(ny[n_index[(i+1)%4]])),(0,255,0),1)
                   
                else:
                    maker = ir_marker()
                    maker_pts = maker.perspective_transform()
                    dist_pts = math.sqrt((maker_pts[0].x-maker_pts[2].x)*(maker_pts[0].x-maker_pts[2].x)+(maker_pts[0].y-maker_pts[2].y)*(maker_pts[0].y-maker_pts[2].y)) 
                    dist_pts = dist_pts + math.sqrt((maker_pts[1].x-maker_pts[3].x)*(maker_pts[1].x-maker_pts[3].x)+(maker_pts[1].y-maker_pts[3].y)*(maker_pts[1].y-maker_pts[3].y))
                    dist_pts /= 2

                dist_det2tub = dist_pts/dist_img*FOCAL_LENGTH                                                
                         
            elif con_num ==2:
                x=[0,0,0,0]
                y=[0,0,0,0]
                ptr=0
                for cnt in contours:
                    (cx,cy), radius = cv2.minEnclosingCircle(cnt)
                    #draw_cross(frame, cx, cy)
                    
                    #highlight detected ir diode by green circle
                    cv2.circle(frame,(int(cx),int(cy)),6,(0,255,0))
                    #print(ptr)
                    x[ptr] = cx
                    y[ptr] = cy
                    ptr = ptr+1
                #special for super b demo video, 2015/9/13
                #extension 90 deg, point 1 higher than point 0
                #screened 2017/01/19
#                 if y[1]<y[0]:
#                     tmp = x[1]
#                     x[1] = x[0]
#                     x[0] = tmp
#                     tmp = y[1]
#                     y[1] = y[0]
#                     y[0] = tmp
#                 dx = x[1]-x[0]
#                 dy = y[1]-y[0]
#                 #print('x1,y1',x[1],y[1])
#                 #print('x0,y0',x[0],y[0])
#                 arc_len = math.sqrt(dx*dx+dy*dy)
#                 dx /= arc_len
#                 dy /= arc_len
#                 #print(dx, dy, arc_len)
#                 coor_list = [0,0,0,0,0,0,0,0]
#                 coor_list[0] = x[1]+arc_len/120*100*dx+arc_len/120*50*dy
#                 coor_list[1] = y[1]+arc_len/120*100*dy-arc_len/120*50*dx
#                 coor_list[2] = coor_list[0]+arc_len/120*370*dy
#                 coor_list[3] = coor_list[1]-arc_len/120*370*dx
#                 coor_list[4] = coor_list[0]+arc_len/120*445*dx
#                 coor_list[5] = coor_list[1]+arc_len/120*445*dy
#                 coor_list[6] = coor_list[2]+arc_len/120*445*dx
#                 coor_list[7] = coor_list[3]+arc_len/120*445*dy                 

                #screened 2017/01/19
#                 cen_x = 0
#                 cen_y = 0
#                 for i in range(0,4):
#                     draw_cross(frame, coor_list[i*2], coor_list[i*2+1], (255,0,0))
#                     cen_x += coor_list[i*2]
#                     cen_y += coor_list[i*2+1]
#                     #print('x,y',i,coor_list[i*2],coor_list[i*2+1])
#                 DELTA_X = cen_x/4
#                 DELTA_Y = cen_y/4
#                 #print(DELTA_X,DELTA_Y)
#                 draw_cross(frame, DELTA_X, DELTA_Y, (0,255,255))
#                 
#                 dist_img = math.sqrt((x[1]-x[0])*(x[1]-x[0])+(y[1]-y[0])*(y[1]-y[0]))
                cv2.circle(frame,(int(ir_pts[4][0]),int(ir_pts[4][1])),6,(0,255,255))
                cv2.circle(frame,(int(ir_pts[5][0]),int(ir_pts[5][1])),6,(0,255,255))
                if GYRO_ON:
                    dist_pts = 1.2
                    if abs(x[0] - x[1]) > abs(y[0] - y[1]):
                        # use x to determine which ir is which
                        if (x[0] - x[1]) * (ir_pts[4][0] - ir_pts[5][0]) < 0:
                            #switch
                            id1 = Vector2(x[1], y[1]) # down ir
                            id2 = Vector2(x[0], y[0])
                        else:
                            id1 = Vector2(x[0], y[0])
                            id2 = Vector2(x[1], y[1])
                    else:
                        # use y to determine which ir is which
                        if (y[0] - y[1]) * (ir_pts[4][1] - ir_pts[5][1]) > 0:
                            #switch
                            id1 = Vector2(x[1], y[1]) # down ir
                            id2 = Vector2(x[0], y[0])
                        else:
                            id1 = Vector2(x[0], y[0])
                            id2 = Vector2(x[1], y[1])
                                  
                        
                    vx = id2 - id1
                    vx.normalize()
                    vy = Vector2(vx[1], -vx[0])
                    dx = x[1]-x[0]
                    dy = y[1]-y[0]
                    arc_len = math.sqrt(dx*dx+dy*dy)
                    s = arc_len / 118.0;
                    det_pts = []
                    det_pts.append(id1 + vx*s*169 + vy*s*6)
                    det_pts.append(det_pts[0] + vx*s*383)
                    det_pts.append(det_pts[1] - vy*s*461)
                    det_pts.append(det_pts[2] - vx*s*383)
                    cen_x = 0
                    cen_y = 0
                    for i in range(0, 4):
                        draw_cross(frame, det_pts[i][0], det_pts[i][1], (0, 0, 255))
                        cen_x += det_pts[i][0]
                        cen_y += det_pts[i][1]
                    DELTA_X = cen_x / 4
                    DELTA_Y = cen_y / 4
                    
                    draw_cross(frame, DELTA_X, DELTA_Y, (0,255,255))
                    dist_img = arc_len
                    

                else:
                    maker = ir_marker()
                    maker_pts = maker.perspective_transform()
                    dist_pts = 0
                    for i in range(0,4):
                        dx = maker_pts[(i+1)%4].x-maker_pts[i].x
                        dy = maker_pts[(i+1)%4].y-maker_pts[i].y
                        dist_pts += math.sqrt(dx*dx+dy*dy)
                    dist_pts /= 4
                dist_det2tub = dist_pts/dist_img*FOCAL_LENGTH  
                    
            else:
                pass
            #cv2.drawContours(frame, contours, -1, (0,255,0))
        
            #central cross
            cv2.line(frame,(0,240),(640,240),(255,255,255),1)
            cv2.line(frame,(320,0),(320,480),(255,255,255),1)
    
            #rotation axis tips
            #cv2.line(frame,(IMAGE_WIDTH-35,10),(IMAGE_WIDTH-35,60),(0,0,255),1)
            #cv2.line(frame,(IMAGE_WIDTH-10,35),(IMAGE_WIDTH-60,35),(0,255,0),1)
            #cv2.putText(frame, 'rotation red axis', (35, 80), font, 0.5, (255,255,255),1,cv2.LINE_AA)
            #cv2.putText(frame, 'rotation green axis', (80, 35), font, 0.5, (255,255,255),1, cv2.LINE_AA)
            
            cv2.putText(frame, 'tub-det distance = %.0f mm'%dist_det2tub, (5, 30), font, 0.5, (255,255,255), 1, cv2.LINE_AA)
            cv2.line(frame,(-SPANX+IMAGE_WIDTH/2,-SPANY+IMAGE_HEIGHT/2),(SPANX+IMAGE_WIDTH/2,-SPANY+IMAGE_HEIGHT/2),(255,255,255),1)
            cv2.line(frame,(-SPANX+IMAGE_WIDTH/2,SPANY+IMAGE_HEIGHT/2),(SPANX+IMAGE_WIDTH/2,SPANY+IMAGE_HEIGHT/2),(255,255,255),1)
            cv2.line(frame,(-SPANX+IMAGE_WIDTH/2,-SPANY+IMAGE_HEIGHT/2),(-SPANX+IMAGE_WIDTH/2,SPANY+IMAGE_HEIGHT/2),(255,255,255),1)
            cv2.line(frame,(SPANX+IMAGE_WIDTH/2,-SPANY+IMAGE_HEIGHT/2),(SPANX+IMAGE_WIDTH/2,SPANY+IMAGE_HEIGHT/2),(255,255,255),1)
            
            cv2.imshow('image',frame)
            cv2.imwrite('c:\\DetectorPosition\\screenshots\\frame%d.jpg'%counter,frame)
            counter += 1
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    if GYRO_ON:    
        reader1.close()
        reader2.close()