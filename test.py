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
        self.pid_angle = PID(0.55, 0, 0.3)
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
            x_power = constrain(int(self.pid_bottom_x.update(pos[0])), -40, 40)
            y_power = int(self.pid_bottom_y.update(pos[1]))
            a_power = int(self.pid_angle.update(ang))
            #print(pos, ang)
            self.mur.set_motor_power(0, y_power + ang)
            self.mur.set_motor_power(1, y_power - ang)
            self.mur.set_motor_power(4, x_power)
            self.keep_depth(depth)
            sleep(0.1)
            if abs(pos[0] - 160) > 10 or abs(pos[1] - 120) > 10 or abs(ang) > 3 or abs(self.mur.get_depth() + depth) > 0.05:
                    offset_timer = self.time()
            if self.time() - offset_timer >= 1.2:
                self.mur.set_motor_power(0, 0)
                self.mur.set_motor_power(1, 0)
                self.mur.set_motor_power(2, 0)
                self.mur.set_motor_power(3, 0)
                self.mur.set_motor_power(4, 0)
                return 0
    def set_obj(self, v, depth, hsv):
        mur = self.mur
        res = self.res
        offset_timer = self.time()
        from time import sleep
        while True:
            pos = v.obj(mur.get_image_bottom(), hsv)
            if pos is None:
                return -1
            x_power = int(self.pid_bottom_x.update(pos[0]))
            y_power = int(self.pid_bottom_y.update(pos[1]))
            #print(pos, ang)
            self.mur.set_motor_power(0, y_power)
            self.mur.set_motor_power(1, y_power)
            self.mur.set_motor_power(4, x_power)
            self.keep_depth(depth)
            sleep(0.1)
            if abs(pos[0] - 160) > 7 or abs(pos[1] - 120) > 7 or abs(self.mur.get_depth() + depth) > 0.05:
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
                self.mur.set_motor_power(0, 0)
                self.mur.set_motor_power(1, 0)
                self.mur.set_motor_power(2, 0)
                self.mur.set_motor_power(3, 0)
                self.mur.set_motor_power(4, 0)
                return -1
            x_power = int(self.pid_bottom_x.update(pos[0]))
            y_power = int(self.pid_bottom_y.update(pos[1]))
            #print(pos, ang)
            self.mur.set_motor_power(0, y_power)
            self.mur.set_motor_power(1, y_power)
            self.mur.set_motor_power(4, x_power)
            self.keep_depth(depth)
            sleep(0.1)
            if abs(pos[0] - 160) > 7 or abs(pos[1] - 120) > 7 or abs(self.mur.get_depth() + depth) > 0.05:
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
            self.mur.set_motor_power(0, 0)
            self.mur.set_motor_power(1, 0)
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
    def obj(self, image, hsv):
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
            # find countour center
            M = cv.moments(cnt)
            #print("M", M["m00"])
            if M["m00"] <= 200:
                continue
            x = M["m10"] / M["m00"]
            y = M["m01"] / M["m00"]
            cnt_center = (int(x), int(y))
            area = cv.contourArea(cnt)
            #cv.circle(img, circle_center, int(radius), (128, 0, 128), 3)
            #box_area = int(box[1][0]*box[1][1])
            #print(int(M["m10"]), int(M["m00"]))
            #print(box)
            #cv.line(img, , 0, (255, 128, 128), 2)
            ##print(box_area / area, "f")
            #cv.drawContours(img,[box],0,(255,0,0),2) # рисуем прямоугольник
            cv.circle(img, cnt_center, 5, (255, 128, 128), 2)
            #print(area)
            cv.imshow("Img", img)
            cv.waitKey(1)
            if area >= 1000:
                return x, y
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

if __name__ == '__main__':
    start_time = time.time()
    from time import sleep
    from sys import exit
    import numpy as np
    orange = (np.array([18, 139, 0]), np.array([21, 255, 255]))
    red = (np.array([7, 97, 0]), np.array([14, 255, 255]))
    green = (np.array([67, 58, 0]), np.array([83, 255, 255]))
    blue = (np.array([128, 86, 0]), np.array([135, 255, 255]))
    yellow = (np.array([23, 95, 0]), np.array([56, 255, 255]))
    catch_time = 1
    ret_time = 9
    a_height = -3.3
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
    cubes = []
    p = 2
    mur = pymurapi.mur_init()
    f = function(mur)
    v = vision(cv)
    #track(mur, f)
    #print(dir(mur))
    '''
    while True:
        f.keep_depth(-3.2)
        print(v.obj(mur.get_image_bottom(), yellow))

    while True:
        img = mur.get_image_bottom()
        cv.imshow("", img)
        cv.waitKey(1)
    '''
    colors = (red, green, blue, yellow)
    scan_cubes = ((green, blue, yellow), (red, blue, yellow), (red,
                                                                             green, yellow), (red, green, blue))
    
    f.set(0, -2.5)
    ang = 0
    color_cube = None
    while True:

        if p == 2:
            ang = 0
            f.set(ang, -2.5, 60)
            p += 1
        elif p == 3:
            timer = time.time()
            while time.time() - timer <= 5:
                v.arrow(mur.get_image_bottom(), orange)
                f.keep_depth(-2.5)
                f.keep_angle(ang)
            while f.set_arrow(v, -3, orange) == -1:
                f.keep_depth(-2.5)
                f.keep_angle(ang)
                pass
            for color in (red, green, blue, yellow):
                while v.arrow(mur.get_image_bottom(), color) is None:
                    f.keep_depth(-2.5)
                    f.keep_angle(ang)
                position.append(quarter(v.arrow(mur.get_image_bottom(), color)[1]))
            print(position)
            break
    for now in range(4):
        p = 0
        while True:
            if p == 0:
                ang = get_angle(position[now])
                f.set(ang, -2.5, 35)
                while True:
                    f.keep_depth(-2.5)
                    f.keep_angle(ang)
                    if v.stand(mur.get_image_bottom(), colors[now]):
                        while f.set_stand(v, -2.8, colors[now]) == -1:
                            pass
                        f.set(depth = -3.2)
                        mur.open_grabber()
                        
                        p += 1
                        break
            elif p == 1:
                color_cube = None
                #print("mode_10")
                while color_cube is None:
                    for i in scan_cubes[now]:
                        pos = v.obj(mur.get_image_bottom(), i)
                        if not pos is None:
                            color_cube = i
                            cubes.append(color_cube)
                while f.set_obj(v, -3.2, color_cube) == -1:
                    pass
                #print("mode_11")
                f.set(mur.get_yaw(), -3.6)
                mur.set_motor_power(0, -5)
                mur.set_motor_power(1, -5)
                time.sleep(0.7)
                mur.close_grabber()
                sleep(catch_time)
                ang = rotate(get_angle(position[now]) - 180)
                f.set(ang, -2.7, 25)
                timer = time.time()
                while time.time() - timer <= ret_time:
                    v.arrow(mur.get_image_bottom(), color_cube)
                    f.keep_depth(-2.7)
                    f.keep_angle(ang)
                #print("mode_14")
                p += 1
            elif p == 2:
                while True:
                    f.keep_depth(-2.7)
                    f.keep_angle(ang)
                    
                    if not v.arrow(mur.get_image_bottom(), color_cube) is None:
                        while f.set_arrow(v, a_height, color_cube) == -1:
                            f.keep_depth(-2.7)
                            f.keep_angle(ang)
                        break
                mur.open_grabber()
                sleep(1)
                f.set(0, -2.8)
                while f.set_arrow(v, -2.8, orange) == -1:
                    pass
                mur.open_grabber()
                print(cubes)
                break
    
    #ang = 0
    for now in range(4):
        p = 0
        if p == 0:
            while f.set_obj(v, -3.3, colors[now]) == -1:
                f.keep_depth(-2.7)
                f.keep_angle(ang)
            f.set(depth=-3.7)
            mur.close_grabber()
            sleep(1)
            f.set(0, -2.8)
            while f.set_arrow(v, -2.8, orange) == -1:
                pass
            p += 1
        if p == 1:
            ang = get_angle(position[now])
            f.set(ang, -2.5, 35)
            timer = time.time()
            while time.time() - timer <= 7:
                v.arrow(mur.get_image_bottom(), orange)
                f.keep_depth(-2.5)
                f.keep_angle(ang)
            while True:
                f.keep_depth(-2.5)
                f.keep_angle(ang)
                if v.stand(mur.get_image_bottom(), colors[now]):
                    while f.set_stand(v, -2.8, colors[now]) == -1:
                        pass
                    f.set(depth=-3.2)
                    mur.open_grabber()
                    sleep(1)
                    p += 1
                    break
        if p == 2:
            ang = rotate(get_angle(position[now]) - 180)
            f.set(ang, -2.7, 25)
            timer = time.time()
            while time.time() - timer <= 7:
                v.arrow(mur.get_image_bottom(), orange)
                f.keep_depth(-2.7)
                f.keep_angle(ang)
    while f.set_arrow(v, -2.5, orange) == -1:
        f.keep_depth(-2.5)
        f.keep_angle(ang)
    f.set(mur.get_yaw(), -2.5, 30)
    while f.set_stand(v, -2.2, orange) == -1:
        f.keep_depth(-2.5)
        f.keep_angle(ang)
    f.set(depth = 0)


print((time.time() - start_time) / 60)
