#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car

Usage:
    manage.py (drive)

Options:
    -h --help          Show this screen.
"""
import os
import time
import numpy as np

from docopt import docopt

import donkeycar as dk
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle
from donkeycar.parts.controller import LocalWebController


def drive(cfg):
    '''
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    '''

    #Initialize car
    V = dk.vehicle.Vehicle()

    print("cfg.CAMERA_TYPE", cfg.CAMERA_TYPE)
    inputs = []
    threaded = True
    from donkeycar.parts.camera import PiCamera
    cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, framerate=cfg.CAMERA_FRAMERATE, vflip=cfg.CAMERA_VFLIP, hflip=cfg.CAMERA_HFLIP)
    V.add(cam, inputs=inputs, outputs=['cam/image_array'], threaded=threaded)

    ctr = LocalWebController(port=cfg.WEB_CONTROL_PORT, mode=cfg.WEB_INIT_MODE)
    V.add(ctr,
      inputs=['cam/image_array'],
      outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
      threaded=True)



    def myConv(mat, pos, mask):
        return np.sum(mat[:, pos - 2:pos + 3, :] * mask)

    def LinePos(mat):
        mask = np.zeros_like(mat[:10, :5, :])
        for i in range(3):
            mask[:, :, i] = np.array([[0., 0.5, 1, 0.5, 0] for i in range(10)])
        score = [myConv(mat, i, mask) for i in range(2, 158)]
        return np.argwhere(score == np.max(score)).flatten()[0]

    
    class MyController:
        '''
        a simple controller class that outputs a constant steering and throttle.
        '''
        def run(self,image):
            if image is None:
                pos=80
            else:
                image_crop=image[110:120,:]
                pos=LinePos(image_crop)
            if pos<60:
                steering=-0.8
            elif pos>100:
                steering=0.8
            else:
                steering=0.0
            throttle = 0.0
            return steering, throttle

    V.add(MyController(), outputs=['angle', 'throttle'])

    #Drive train setup
    steering_controller = PCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    steering = PWMSteering(controller=steering_controller,
                                    left_pulse=cfg.STEERING_LEFT_PWM, 
                                    right_pulse=cfg.STEERING_RIGHT_PWM)
    
    throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    throttle = PWMThrottle(controller=throttle_controller,
                                    max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                    zero_pulse=cfg.THROTTLE_STOPPED_PWM, 
                                    min_pulse=cfg.THROTTLE_REVERSE_PWM)

    V.add(steering, inputs=['angle'])
    V.add(throttle, inputs=['throttle'])
    
    #run the vehicle for 20 seconds
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ, 
            max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()
    
    if args['drive']:
        drive(cfg)
