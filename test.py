import pymurapi
import time
import cv2 as cv
'''
Underwater power 2019
Written by Yaroslav Savelev
https://github.com/Yaroslav21Savelev
'''
class PID:
    def __init__(self, P, I, D):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            
        return self.output;
        
    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

    def setSetPoint(self, set_point):
        self.SetPoint = set_point

def constrain(v, min, max):
    if v < min:
        return min
    if v > max:
        return max
    return v
def rotate(val):
    if val >= 180:
        return val - 360
    elif val <= -180:
        return val + 360
    else:
        return val
class function():
    def __init__(self, mur):
        self.mur = mur
        from time import time
        self.time = time
        self.speed = 0
        self.angle = 0
        self.depth = 0
        self.res = (320, 240)
        self.depth_timer = self.time()
        self.angle_timer = self.time()
        self.pid_depth = PID(50, 0, 50)
        self.pid_angle = PID(0.55, 0, 0.3)
        self.pid_bottom_x = PID(0.5, 0.3, 0.3)
        self.pid_bottom_y = PID(0.47, 0.3, 0.3)
        self.pid_angle.setSetPoint(0)
        self.pid_bottom_x.setSetPoint(160)
        self.pid_bottom_y.setSetPoint(120)
    def set_arrow(self, v, depth):
        mur = self.mur
        res = self.res
        from time import sleep
        while True:
            data = v.arrow(mur.get_image_bottom())
            if data is None:
                self.mur.set_motor_power(0, 0)
                self.mur.set_motor_power(1, 0)
                self.mur.set_motor_power(2, 0)
                self.mur.set_motor_power(3, 0)
                self.mur.set_motor_power(4, 0)
                return None
            pos, ang = data
            x_power = int(self.pid_bottom_x.update(pos[0]))
            y_power = int(self.pid_bottom_y.update(pos[1]))
            a_power = int(self.pid_angle.update(ang))
            #print(pos, ang)
            self.mur.set_motor_power(0, y_power + ang)
            self.mur.set_motor_power(1, y_power - ang)
            self.mur.set_motor_power(4, x_power)
            self.keep_depth(depth)
            sleep(0.1)
            if abs(pos[0] - 160) <= 5 and  abs(pos[1] - 120) <= 5 and abs(ang) <= 3 and abs(self.mur.get_depth() + depth) <= 0.05:
                self.mur.set_motor_power(0, 0)
                self.mur.set_motor_power(1, 0)
                self.mur.set_motor_power(2, 0)
                self.mur.set_motor_power(3, 0)
                self.mur.set_motor_power(4, 0)
                return 0
    def keep_depth(self, target):        
        if target is None:
            self.mur.set_motor_power(2, 0)
            self.mur.set_motor_power(3, 0)
        if (self.time() - self.depth_timer) >= 0.2:
            out = -int(self.pid_depth.update(self.mur.get_depth() + target))
            '''
            if out >= 0:
                out += 20
            else:
                out -= 20
            '''
            self.mur.set_motor_power(2, constrain(int(out), -100, 100))
            self.mur.set_motor_power(3, constrain(int(out), -100, 100))
            self.depth_timer = self.time()
    def keep_angle(self, target, speed = 0):
        if target is None:
            self.mur.set_motor_power(0, 0)
            self.mur.set_motor_power(1, 0)
        if (self.time() - self.angle_timer) >= 0.2 and (not self.angle is None):
            angle_l = rotate(mur.get_yaw() - target)
            out = int(self.pid_angle.update(angle_l))
            #print(out)
            mur.set_motor_power(0, constrain(out + self.speed, -100, 100))
            mur.set_motor_power(1, constrain(-out + self.speed, -100, 100))
            self.angle_timer = self.time()
    def set(self, angle = None, depth = None, speed = 0):
        offset_timer = self.time()
        self.speed = 0
        self.angle = angle
        self.depth = depth
        from time import sleep
        while True:
            if not depth is None:
                if abs(self.mur.get_depth() + depth) >= 0.05:
                    offset_timer = self.time()
            if not angle is None:
                if abs(self.mur.get_yaw() - angle) >= 3:
                    offset_timer = self.time()
            if self.time() - offset_timer >= 1:
                break
            self.keep_angle(angle)
            self.keep_depth(depth)
            #print("setting")
            sleep(0.1)
        self.speed = speed
class vision():
    
    def __init__(self, cv):
        import numpy
        self.cv = cv
        self.np = numpy
        
        '''
        def nothing(self):
            pass
        window_name = "track_img"
        cv.namedWindow(window_name)
        
        cv.createTrackbar('UH',window_name,0,255,nothing)
        cv.setTrackbarPos('UH',window_name, 255)

        cv.createTrackbar('US',window_name,0,255,nothing)
        cv.setTrackbarPos('US',window_name, 255)

        cv.createTrackbar('UV',window_name,0,255,nothing)
        cv.setTrackbarPos('UV',window_name, 255)

        # create trackbars for Lower HSV
        cv.createTrackbar('LH',window_name,0,255,nothing)
        cv.setTrackbarPos('LH',window_name, 0)

        cv.createTrackbar('LS',window_name,0,255,nothing)
        cv.setTrackbarPos('LS',window_name, 0)

        cv.createTrackbar('LV',window_name,0,255,nothing)
        cv.setTrackbarPos('LV',window_name, 0)
        '''
    '''
    def detect_red_objects(self, image):
        np = self.np
        cv = self.cv
        img_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        lower_red = np.array([0,50,50])
        upper_red = np.array([10,255,255])
        mask0 = cv.inRange(img_hsv, lower_red, upper_red)

        lower_red = np.array([170,50,50])
        upper_red = np.array([180,255,255])
        mask1 = cv.inRange(img_hsv, lower_red, upper_red)

        mask = mask0 + mask1
        
        contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(image, contours, -1, (128,0,0), 1)
        
        for cnt in contours:
            (x,y),radius = cv.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            cv.circle(image, center, int(3), (128, 0, 128))
        to_draw = image.copy()                    
        cv.imshow("Image", to_draw)
        cv.waitKey(5)

    def detect_rectangle(self, image):
        np = self.np
        cv = self.cv
        img = image.copy()
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        img = cv.GaussianBlur(img, (5, 5), 2)
        ret, cannyParams = cv.threshold(img,  0, 255, cv.THRESH_BINARY_INV)
        #print(ret)
        #print(*cannyParams)
        #cv.Canny()
        #img = cv.Canny(img, cannyParams, cannyParams / 2.0)
        img = cv.Canny(img, ret, ret / 2)
        contours = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        #print(*contours)
        for i in contours:
            if i is None:
                continue
            
            if len(i) < 5:
                continue
            #print(cv.contourArea(i[0]))
            if (cv.contourArea(i[0]) < 300.0):
                continue
            bEllipse = cv.fitEllipse(i[0])
            hull = cv.convexHull(i[0], True)
            hull = cv.approxPolyDP(hull, 15, True);
            if (not cv.isContourConvex(hull)):
                continue
            #print(hull)
            bEllipse = map(int, bEllipse)
            if len(hull) == 4:
                #print("Found")
                return((bEllipse[0][0], bEllipse[0][1]), bEllipse[2])
        return None
    '''
    def get_corners(self, image):
        np = self.np
        cv = self.cv
        img = image.copy()
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        img = cv.GaussianBlur(img, (5, 5), 2)
        ret, cannyParams = cv.threshold(img,  0, 255, cv.THRESH_BINARY_INV)
        img = cv.Canny(img, ret, ret / 2)
        contours = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        for i in contours:
            if i is None:
                continue
            if len(i) < 5:
                continue
            if (cv.contourArea(i[0]) < 300.0):
                continue
            bEllipse = cv.fitEllipse(i[0])
            hull = cv.convexHull(i[0], True)
            hull = cv.approxPolyDP(hull, 15, True);
            if (not cv.isContourConvex(hull)):
                continue
            bEllipse = map(int, bEllipse)
            return len(hull)
        return None
    def arrow(self, image):
        np = self.np
        cv = self.cv
        orange_l = self.np.array([16, 141, 0])
        orange_u = self.np.array([23, 255, 208])
        img = cv.GaussianBlur(image, (5,5), cv.BORDER_DEFAULT)
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(img_hsv, orange_l, orange_u)
        contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cv.imshow("Mask", mask)
        cv.waitKey(1)
        for cnt in contours:
            # find countour center
            M = cv.moments(cnt)
            #print("M", M["m00"])
            if M["m00"] <= 200:
                continue
            x = M["m10"] / M["m00"]
            y = M["m01"] / M["m00"]
            cnt_center = (x, y)
            # find circle center
            (x,y),radius = cv.minEnclosingCircle(cnt)
            circle_center = (x, y)
            # find angel of dst(circle_center, cnt_center)
            from math import atan2, pi
            p1, p2 = circle_center, cnt_center
            ang = atan2(p1[1] - p2[1], p1[0] - p2[0]) * 180 / pi - 90
            # rotate angle to global coord
            if ang < -180:
                ang += 360
            return cnt_center, ang
        
    def track(self, image):
        np = self.np
        cv = self.cv
        window_name = "track_img"
        uh = cv.getTrackbarPos('UH',window_name)
        us = cv.getTrackbarPos('US',window_name)
        uv = cv.getTrackbarPos('UV',window_name)
        upper_blue = np.array([uh,us,uv])
        # get current positions of Lower HSCV trackbars
        lh = cv.getTrackbarPos('LH',window_name)
        ls = cv.getTrackbarPos('LS',window_name)
        lv = cv.getTrackbarPos('LV',window_name)
        upper_hsv = np.array([uh,us,uv])
        lower_hsv = np.array([lh,ls,lv])
        print(upper_hsv, lower_hsv)
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, lower_hsv, upper_hsv)
        cv.imshow("HSV", mask)
        cv.waitKey(5)
    

if __name__ == '__main__':
    from time import sleep
    from sys import exit
    mur = pymurapi.mur_init()
    f = function(mur)
    v = vision(cv)
    f.set(0, -2.5, 30)
    while True:
        f.keep_depth(-2.5)
        f.keep_angle(0)
        if not v.arrow(mur.get_image_bottom()) is None:
            while f.set_arrow(v, -2.5) == -1:
                pass
            f.set(mur.get_yaw(), -0.5)
            break
