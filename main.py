# Данный пример предназначен для работы в семуляторе!  

# В данном примере реализован простой пропорциональный регулятор по курсу.
# Аппарат будет плыть прямо с тягой в 60%, удерживая курс в -70 градусов, 30 секунд.

import pymurapi as mur
import cv2 as cv
import cv2
import time
import numpy as np
from math import *
red = (np.array([7, 97, 0]), np.array([14, 255, 255]))
center_width = 320/2
center_heigth = 240/2 - 25
class mur_object:
    center,size,x,y = 0,0,-1,-1
    def _init_():
        self.center = (0,0)
        self.size = 0
        self.x = -1
        self.y = -1
        
     

auv = mur.mur_init()

# Перевод угла >< 360 в 0 <=> 360
def clamp_to_360(angle):
    if angle < 0.0:
        return angle + 360.0
    if angle > 360.0:
        return angle - 360.0
    return angle

# Перевод угла из 0 <=> 360 в -180 <=> 180
def to_180(angle):
    if angle > 180.0:
        return angle - 360.0
    return angle

# Преобразовать v в промежуток между min max
def clamp(v, min, max):
	if v < min:
		return min
	if v > max:
		return max
	return v

# Функция удержания курса
def keep_yaw(yaw_to_set, power):
    current_yaw = auv.get_yaw()
    er = clamp_to_360(yaw_to_set - current_yaw)
    er = to_180(er)
    res = er * -0.8
    auv.set_motor_power(0, clamp(int(power - res), -100, 100))
    auv.set_motor_power(1, clamp(int(power + res), -100, 100))

def keep_depth(depth_to_set):
    motor = 250 * (auv.get_depth() - depth_to_set)    
    #auv.set_motor_power(0, clamp(int(motor), -100, 100))
    auv.set_motor_power(3, clamp(int(motor), -100, 100))
                   
def detect_object(image,hsv):
    from math import pi
        
        
    hsv_l = hsv[0]
    hsv_u = hsv[1]
    img = image.copy()
    simg = cv.GaussianBlur(img, (5, 5), cv.BORDER_DEFAULT)
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(img_hsv, hsv_l, hsv_u)
    contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cv.imshow("Mask", mask)
    cv.waitKey(1)
    o2ret = mur_object()
    for cnt in contours:
            # find countour center
        #M = cv.moments(cnt)
            #print("M", M["m00"])
        #if M["m00"] <= 200:
        #    continue
       # x = M["m10"] / M["m00"]
        #y = M["m01"] / M["m00"]
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        cnt_center = (int(x), int(y))
        area = cv.contourArea(cnt)
            #cv.circle(img, circle_center, int(radius), (128, 0, 128), 3)
            #box_area = int(box[1][0]*box[1][1])
            #print(int(M["m10"]), int(M["m00"]))
            #print(box)
            #cv.line(img, , 0, (255, 128, 128), 2)
            ##print(box_area / area, "f")
       
        
            #print(area)
        cv.imshow("Img", img)
        cv.waitKey(1)
        if area >= 100:
            o2ret.x = int(x)
            o2ret.y = int(y)
            o2ret.size = int(radius)
            o2ret.center = (int(x),int(y))
            cv.circle(img, cnt_center, int(radius), (255, 128, 128), 2)
        
            return o2ret
    return o2ret

   
  



def centeringXZ(obj, epsilon):
    

    x = obj.x
    y = obj.y
    p,pD = 0,0
    k1, k2 = 0.1,0.1 
    epsilon = round(epsilon * (5-auv.get_depth()))
    #print(epsilon)
   # if (x < (center_width - epsilon) ):
    #   pD = k1*( center_width - (x))
   # if (x > (center_width + epsilon) ):
    pD = k1*( center_width - (x))
    if abs(x - center_width) <= epsilon*3:
       pD = 0

    #if (y < (center_heigth - epsilon)): 
     #  p = -k2*(center_heigth - y)
    #if (y > (center_heigth + epsilon)): 
    p = k2*(center_heigth - y)
    if abs(y - center_heigth) <= epsilon:
       pD = 0
    print(abs(y - center_heigth),abs(x - center_width))
    if abs(y - center_heigth) <= epsilon and abs(x - center_width) <= epsilon*3:
       auv.set_motor_power(4, 0)
       auv.set_motor_power(0, 0)
       auv.set_motor_power(1, 0)
       
       return True
          
  
    auv.set_motor_power(4, pD)
    auv.set_motor_power(0, p)
    auv.set_motor_power(1, p)
   
    return False
    
    
if __name__ == '__main__':
    now = time.time()
    f = False
    while not f:
           time.sleep(0.01)
           #keep_depth(2)
           a = detect_object(auv.get_image_bottom(), red)
           print(a.x,a.y,a.size)
           #print(a.size,a.center)
           if a.size > 0:
               f = centeringXZ(a,3)
           else:
               auv.set_motor_power(4, 0)
               auv.set_motor_power(0, 0)
               auv.set_motor_power(1, 0) 
    a = detect_object(auv.get_image_bottom(), red)
    while a.size < 1200:
       auv.set_motor_power(3,-10)
       a = detect_object(auv.get_image_bottom(), red)
       time.sleep(0.01)
      
  
           
              
             
    print('Centered!')
    
                                                             