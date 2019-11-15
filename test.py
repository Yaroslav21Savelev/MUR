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
    if val >= 360:
        return val - 360
    elif val < 0:
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
        self.depth_timer = self.time()
        self.angle_timer = self.time()
        self.pid_depth = PID(0.95, 0, 0.96)
        self.pid_angle = PID(0.95, 0, 0.96)
        self.pid_angle.setSetPoint(180)
    def keep_depth(self, target):
        if target is None:
            self.mur.set_motor_power(2, 0)
        #print(self.time() - self.depth_timer)
        if (self.time() - self.depth_timer) >= 0.1 and (not self.depth is None):
            out = int(self.pid_depth.update(self.mur.get_depth() - target))
            if out >= 0:
                out += 20
            else:
                out -= 20
            #print(out)
            self.mur.set_motor_power(2, constrain(int(out), -100, 100))
            self.depth_timer = self.time()
    def keep_angle(self, target, speed = 0):
        if target is None:
            self.mur.set_motor_power(0, 0)
            self.mur.set_motor_power(1, 0)
        if (self.time() - self.angle_timer) >= 0.1 and (not self.angle is None):
            angle_l = rotate(mur.get_yaw() - target + 180)
            out = int(self.pid_angle.update(angle_l))
            #print(out)
            mur.set_motor_power(0, constrain(-out - self.speed, -100, 100))
            mur.set_motor_power(1, constrain(out - self.speed, -100, 100))
            self.angle_timer = self.time()
    def set(self, angle = None, depth = None, speed = 0):
        offset_timer = self.time()
        self.speed = 0
        self.angle = angle
        self.depth = depth
        while True:
            if not depth is None:
                if abs(self.mur.get_depth() - depth) >= 3:
                    offset_timer = self.time()
            if not angle is None:
                if abs(self.mur.get_yaw() - angle) >= 3:
                    offset_timer = self.time()
            if self.time() - offset_timer >= 1:
                break
            self.keep_angle(angle)
            self.keep_depth(depth)
            #print("setting")
        self.speed = speed
class vision():
    def __init__(self, cv):
        import numpy
        self.cv = cv
        self.np = numpy
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
if __name__ == '__main__':
    from time import sleep
    mur = pymurapi.mur_init()
    f = function(mur)
    v = vision(cv)
    f.set(20, 80, 10)
    while True:
        '''
        #v.detect_red_objects(mur.get_image_bottom())
        img = mur.get_image_bottom()
        out = v.detect_rectangle(mur.get_image_bottom())
        if not out is None:
            img = cv.circle(img, out[0], 2, (0, 255, 0), 3)
            print(out)
        f.keep_depth(80)
        f.keep_angle(20)
        cv.imshow("",img)
        cv.waitKey(1)
        '''
        f.keep_depth(80)
        f.keep_angle(20)
        sleep(0.1)