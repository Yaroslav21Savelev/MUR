import numpy as np
import pymurapi
import time
import cv2 as cv
'''
Underwater power 2019
'''
class PID:
    def __init__(self, P, I, D):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.SetPoint = 0.0
        self.clear()

    def clear(self):
        #self.SetPoint = 0.0

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
        self.pid_angle = PID(0.55, 0, 0.18)
        self.pid_bottom_x = PID(0.33, 0.3, 0.3)
        self.pid_bottom_y = PID(0.3, 0.3, 0.3)
        self.pid_angle.setSetPoint(0)
        self.pid_bottom_x.setSetPoint(160)
        self.pid_bottom_y.setSetPoint(120)
    def set_arrow(self, v, depth, hsv):
        mur = self.mur
        res = self.res
        from time import sleep
        offset_timer = self.time()
        
        while True:
            data = v.arrow(mur.get_image_bottom(), hsv)
            if data is None:
                return -1
            pos, ang = data
            x_power = constrain(int(self.pid_bottom_x.update(pos[0])), -20, 20)
            y_power = constrain(int(self.pid_bottom_y.update(pos[1])), -20, 20)
            a_power = constrain(int(self.pid_angle.update(ang)), -20, 20)
            #print(pos, ang)
            self.mur.set_motor_power(0, constrain(y_power + ang, -20, 20))
            self.mur.set_motor_power(1, constrain(y_power - ang, -20, 20))
            self.mur.set_motor_power(4, constrain(x_power, -20, 20))
            self.keep_depth(depth)
            sleep(0.1)
            if abs(pos[0] - 160) > 15 or abs(pos[1] - 120) > 15 or abs(ang) > 3 or abs(self.mur.get_depth() + depth) > 0.05:
                    offset_timer = self.time()
            if self.time() - offset_timer >= 1.2:
                self.mur.set_motor_power(0, 0)
                self.mur.set_motor_power(1, 0)
                self.mur.set_motor_power(2, 0)
                self.mur.set_motor_power(3, 0)
                self.mur.set_motor_power(4, 0)
                return 0
    def set_obj(self, v, depth, hsv, x = 0, y = 0):
        mur = self.mur
        res = self.res
        offset_timer = self.time()
        from time import sleep
        while True:
            pos = v.obj(mur.get_image_bottom(), hsv)
            if pos is None:
                return -1

            x_power = constrain(int(self.pid_bottom_x.update(pos[0] + x)), -20, 20)
            y_power = constrain(int(self.pid_bottom_y.update(pos[1] + y)), -20, 20)
            #print(pos, ang)
            self.mur.set_motor_power(0, y_power)
            self.mur.set_motor_power(1, y_power)
            self.mur.set_motor_power(4, x_power)
            self.keep_depth(depth)
            sleep(0.1)
            if abs(pos[0] - 160 + x) > 5 or abs(pos[1] - 120 + y) > 5 or abs(self.mur.get_depth() + depth) > 0.05:
                    offset_timer = self.time()
            if self.time() - offset_timer >= 1:
                self.mur.set_motor_power(0, 0)
                self.mur.set_motor_power(1, 0)
                self.mur.set_motor_power(2, 0)
                self.mur.set_motor_power(3, 0)
                self.mur.set_motor_power(4, 0)
                return 0
    def set_stand(self, v, depth, hsv):
        mur = self.mur
        res = self.res
        offset_timer = self.time()
        from time import sleep
        while True:
            pos = v.stand(mur.get_image_bottom(), hsv)
            if pos is None:
                return -1
            x_power = constrain(int(self.pid_bottom_x.update(pos[0])), -20, 20)
            y_power = constrain(int(self.pid_bottom_y.update(pos[1])), -20, 20)
            #print(pos, ang)
            self.mur.set_motor_power(0, y_power)
            self.mur.set_motor_power(1, y_power)
            self.mur.set_motor_power(4, x_power)
            self.keep_depth(depth)
            sleep(0.1)
            if abs(pos[0] - 160) > 20 or abs(pos[1] - 120) > 20 or abs(self.mur.get_depth() + depth) > 0.05:
                    offset_timer = self.time()
            if self.time() - offset_timer >= 1:
                self.mur.set_motor_power(0, 0)
                self.mur.set_motor_power(1, 0)
                self.mur.set_motor_power(2, 0)
                self.mur.set_motor_power(3, 0)
                self.mur.set_motor_power(4, 0)
                return 0
    def keep_depth(self, target):
        self.mur.get_image_bottom()
        if target is None:
            self.mur.set_motor_power(2, 0)
            self.mur.set_motor_power(3, 0)
        elif (self.time() - self.depth_timer) >= 0.2:
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
        self.mur.get_image_bottom()
        if target is None:
            mur.set_motor_power(0, 0)
            mur.set_motor_power(1, 0)
        elif (self.time() - self.angle_timer) >= 0.2 and (not self.angle is None):
            angle_l = rotate(mur.get_yaw() - target)
            out = int(self.pid_angle.update(angle_l))
            #print(out)
            mur.set_motor_power(0, constrain(out + self.speed, -100, 100))
            mur.set_motor_power(1, constrain(-out + self.speed, -100, 100))
            self.angle_timer = self.time()
    def set(self, angle = None, depth = None, speed = 0):
        offset_timer = self.time()
        self.speed = 0
        from time import sleep
        self.pid_bottom_x.clear()
        self.pid_bottom_y.clear()
        self.pid_angle.clear()
        while True:
            if not depth is None:
                if abs(self.mur.get_depth() + depth) >= 0.05:
                    offset_timer = self.time()
            if not angle is None:
                if abs(abs(self.mur.get_yaw()) - abs(angle)) >= 3:
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

    def obj(self, image, hsv):
        from math import pi
        np = self.np
        cv2 = self.cv
        cv = self.cv
        hsv_l = hsv[0]
        hsv_u = hsv[1]
        img = image.copy()
        img = cv.GaussianBlur(img, (5, 5), cv.BORDER_DEFAULT)
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask_ = cv.inRange(img_hsv, hsv_l, hsv_u)

        contours, hierarchy = cv2.findContours(mask_, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        n = 0
        for i in contours:
            size = cv2.contourArea(i)
            rect = cv2.minAreaRect(i)
            if 100 < size < 10000:
                if len(i) < 5:
                    continue
                bEllipse = cv.fitEllipse(i)
                hull = cv.convexHull(i, True)
                hull = cv.approxPolyDP(hull, 15, True)
                if (not cv.isContourConvex(hull)):
                    continue
                #angle - bEllipse[2]
                
                if len(hull) == 4:
                    cv2.imshow('Mask', mask_)
                    cv2.waitKey(1)
                    return int(bEllipse[0][0]), int(bEllipse[0][1])
                
    def arrow(self, image, hsv):
        np = self.np
        cv = self.cv
        hsv_l = hsv[0]
        hsv_u = hsv[1]
        img = image.copy()
        img = cv.GaussianBlur(img, (5, 5), cv.BORDER_DEFAULT)
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(img_hsv, hsv_l, hsv_u)
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
    def stand(self, image, hsv):
        from math import pi
        np = self.np
        cv = self.cv
        hsv_l = hsv[0]
        hsv_u = hsv[1]
        img = image.copy()
        img = cv.GaussianBlur(img, (5, 5), cv.BORDER_DEFAULT)
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(img_hsv, hsv_l, hsv_u)
        contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cv.imshow("Mask", mask)
        cv.waitKey(1)
        for cnt in contours:
            # find circle center
            (x,y),radius = cv.minEnclosingCircle(cnt)
            circle_center = (x, y)
            area = cv.contourArea(cnt)
            #cv.circle(img, circle_center, int(radius), (128, 0, 128), 3)
            if area != 0:
                if radius * (pi ** 2) / area <= 0.15:
                    return circle_center
                #print(circle_center)
        #cv.imshow("Img", img)
        #cv.waitKey(1)
def track(mur, f):
    global cv
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
    while True:
        image = mur.get_image_bottom()
        f.keep_depth(-2.5)
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


orange = np.array([np.array([18, 139, 0]), np.array([21, 255, 255])])
red = np.array([np.array([7, 97, 0]), np.array([14, 255, 255])])
green = np.array([np.array([67, 58, 0]), np.array([83, 255, 255])])
blue = np.array([np.array([128, 86, 0]), np.array([135, 255, 255])])
yellow = np.array([np.array([23, 95, 0]), np.array([56, 255, 255])])
def quarter(ang):
    '''
    1   2
      r
    3   4
    '''
    if ang >= -90 and ang < 0:
        return 1
    elif ang >= 0 and ang < 90:
        return 2
    elif ang >= -180 and ang < -90:
        return 3
    elif ang >= 90 and ang <= 180:
        return 4
def get_angle(quarter):
    arr = [None, -45, 45, -135, 135]
    return arr[quarter]
def nextDir(now, nex, positions):
    '''
    1   2
      r
    3   4
    '''
    if (now == red).all():
        now = positions[0]
    elif (now == green).all():
        now = positions[1]
    elif (now == blue).all():
        now = positions[2]
    elif (now == yellow).all():
        now = positions[3]
    if (nex == red).all():
        nex = positions[0]
    elif (nex == green).all():
        nex = positions[1]
    elif (nex == blue).all():
        nex = positions[2]
    elif (nex == yellow).all():
        nex = positions[3]
    elif (nex == orange).all():
        nex = 0
    print(now, nex)
    if now == 1:
        if nex == 0:
            return 135
        elif nex == 2:
            return 90
        elif nex == 3:
            return 180
        elif nex == 4:
            return 135
    elif now == 2:
        if nex == 0:
            return -135
        elif nex == 1:
            return -90
        elif nex == 3:
            return -135
        elif nex == 4:
            return 180
    elif now == 3:
        if nex == 0:
            return 45
        elif nex == 1:
            return 0
        elif nex == 2:
            return 45
        elif nex == 4:
            return 90
    elif now == 4:
        if nex == 0:
            return -45
        elif nex == 1:
            return -45
        elif nex == 2:
            return 0
        elif nex == 3:
            return -90


if __name__ == '__main__':
    start_time = time.time()
    from time import sleep
    from sys import exit
    import numpy as np
    
    catch_time = 1
    ret_time = 9
    a_height = -3.3
    t_speed = 48
    # r, g, b, y
    def get_color(index):
        if index == 0:
            return red
        if index == 1:
            return green
        if index == 2:
            return blue
        if index == 3:
            return yellow
    position = []
    cubes = np.array([orange])
    print(cubes)
    mur = pymurapi.mur_init()
    f = function(mur)
    v = vision(cv)
    #track(mur, f)
    #print(dir(mur))
    '''
    while True:
        f.keep_depth(-2.5)
        print(v.get_corners(mur.get_image_bottom(), yellow))
        #print(v.obj(mur.get_image_bottom(), yellow))
    '''
    colors = np.array([red, green, blue, yellow])
    f.set(0, -2.5)
    ang = 0
    color_cube = None
    ############### Start, get positions of stands
    ang = 0
    f.set(ang, -2.5, 50)
    timer = time.time()
    while time.time() - timer <= 5:
        v.arrow(mur.get_image_bottom(), orange)
        f.keep_depth(-2.5)
        f.keep_angle(ang)
    while f.set_arrow(v, -2.5, orange) == -1:
        f.keep_depth(-2.5)
        f.keep_angle(ang)
    while True:
        for c in colors:
            while v.arrow(mur.get_image_bottom(), c) is None:
                f.keep_depth(-2.5)
                f.keep_angle(ang)
                sleep(0.1)
            position.append(quarter(v.arrow(mur.get_image_bottom(), c)[1]))
        print(position)
        if len(set(position)) == len(position):
            break
        else:
            position.clear()
    ###############

    ang = get_angle(position[0])
    f.set(ang, -2.5, 50)
    while True:
        f.keep_depth(-2.5)
        f.keep_angle(ang)
        if v.stand(mur.get_image_bottom(), red):
            while f.set_stand(v, -2.8, red) == -1:
                pass
            f.set(depth=-3.2)
            mur.open_grabber()
            break
    color_cube = None
    while color_cube is None:
        for i in colors:
            pos = v.obj(mur.get_image_bottom(), i)
            if not pos is None and  not (np.isin(i, cubes)).all():
                color_cube = i
    cubes = np.vstack((cubes, [color_cube]))
    print(cubes)
    while f.set_obj(v, -3.2, color_cube, y = 15, x = 10) == -1:
        pass
    f.set(depth = -3.65)
    time.sleep(0.7)
    mur.close_grabber()
    time.sleep(1.2)
    ang = nextDir(red, color_cube, position)
    f.set(ang, -2.5, t_speed)
    timer = time.time()
    while time.time() - timer <= 4:
        v.stand(mur.get_image_bottom(), color_cube)
        f.keep_depth(-2.5)
        f.keep_angle(ang)
    
    ############################
    for j in range(3):
        last = color_cube
        color_cube = None
        while True:
            f.keep_depth(-2.5)
            f.keep_angle(ang)
            if v.stand(mur.get_image_bottom(), last):
                while f.set_stand(v, -2.8, last) == -1:
                    pass
                f.set(depth = -3.1)
                break
        while color_cube is None:
            for i in colors:
                pos = v.obj(mur.get_image_bottom(), i)
                if not pos is None and not (np.isin(i, cubes)).all():
                    color_cube = i
        cubes = np.vstack((cubes, [color_cube]))
        print(cubes)
        while f.set_obj(v, -3.1, color_cube, x = -80, y = 60) == -1:
            pass
        f.set(depth = -3.55)
        mur.open_grabber()
        sleep(1)
        f.set(depth = -3.1)
        while f.set_obj(v, -3.1, color_cube, y = 15, x = 10) == -1:
            pass
        f.set(depth = -3.65)
        time.sleep(0.7)
        mur.close_grabber()
        time.sleep(1.2)
        ang = nextDir(last, color_cube, position)
        f.set(ang, -2.5, t_speed)
        timer = time.time()
        while time.time() - timer <= 7:
            v.stand(mur.get_image_bottom(), color_cube)
            f.keep_depth(-2.5)
            f.keep_angle(ang)
    last = color_cube
    color_cube = None   
    while True:
        f.keep_depth(-2.5)
        f.keep_angle(ang)
        if v.stand(mur.get_image_bottom(), last):
            while f.set_stand(v, -2.8, last) == -1:
                pass
            f.set(depth=-3.55)
            mur.open_grabber()
            sleep(1)
            break
    f.set(depth = -2.5)

    ang = nextDir(last, orange, position)
    f.set(ang, -2.5, 30)
    timer = time.time()
    while time.time() - timer <= 7:
            v.arrow(mur.get_image_bottom(), orange)
            f.keep_depth(-2.5)
            f.keep_angle(ang)
    while f.set_arrow(v, -2.5, orange) == -1:
        f.keep_depth(-2.5)
        f.keep_angle(ang)
    f.set(0, -2.5, t_speed)
    while f.set_stand(v, -2.2, orange) == -1:
        f.keep_depth(-2.5)
        f.keep_angle(0)
    f.set(0, -0.05, 0)
print((time.time() - start_time) / 60)
