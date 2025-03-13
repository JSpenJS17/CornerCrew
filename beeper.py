
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from ros_robot_controller_msgs.msg import BuzzerState
from ros_robot_controller_msgs.msg import MotorState, MotorsState
import math
import signal, sys

# table that converts notes to frequencies
note_freqs = {
    "C"  : [    16.35, 	32.7, 	65.41, 	130.81, 261.63,	523.25,	1046.5,  2093, 	    4186    ],
    "C#" : [    17.32, 	34.65, 	69.3, 	138.59, 277.18,	554.37,	1108.73, 2217.46, 	4434.92 ],
    "D"  : [    18.35, 	36.71, 	73.42, 	146.83, 293.66,	587.33,	1174.66, 2349.32, 	4698.63 ],
    "D#" : [    19.45, 	38.89, 	77.78, 	155.56, 311.13,	622.25,	1244.51, 2489, 	    4978    ],
    "E"  : [    20.6, 	41.2, 	82.41, 	164.81, 329.63,	659.25,	1318.51, 2637, 	    5274    ],
    "F"  : [    21.83, 	43.65, 	87.31, 	174.61, 349.23,	698.46,	1396.91, 2793.83, 	5587.65 ],
    "F#" : [    23.12, 	46.25, 	92.5, 	185, 	369.99,	739.99,	1479.98, 2959.96, 	5919.91 ],
    "G"  : [    24.5, 	49,	    98, 	196, 	392, 	783.99,	1567.98, 3135.96, 	6271.93 ],
    "G#" : [    25.96, 	51.91, 	103.83, 207.65, 415.3, 	830.61,	1661.22, 3322.44, 	6644.88 ],
    "A"  : [    27.5, 	55, 	110, 	220, 	440, 	880,    1760,    3520, 	    7040    ],
    "A#" : [    29.14, 	58.27, 	116.54, 233.08, 466.16,	932.33,	1864.66, 3729.31, 	7458.62 ],
    "B"  : [    30.87, 	61.74, 	123.47, 246.94, 493.88,	987.77,	1975.53, 3951, 	    7902.13 ],
    "R"  : [    0,      0,      0,      0,      0,      0,      0,       0,         0       ]
}

class Note:
    """Data class with 3 fields: on_time, off_time, and note_freq.
    bpm is the bpm of the note
    on_time is the seconds the buzz is played for
    off_time is equal to on time, so each noise played is 7/16 on and 9/16 off
    note_freq is an int representing the frequency of each note"""

    def __init__(self, bpm, note, slur = False):
        """bpm: beats per minute that the note is played at
        note: tuple of (note_str, octave) from table above i.e. ("A", 2)"""
        total_time = 60.0 / bpm
        self.bpm = bpm

        if slur:
            self.on_time = total_time
        else:
            self.on_time = (total_time/16.0) * 7

        self.off_time = total_time - self.on_time
        self.freq = int(note_freqs[note[0]][note[1]])
        

# TODO: ADD HALF AND EIGTH NOTES TO THIS SONG
# define Star Spangled Banner Notes (!!! NEEDS TO INCLUDE HALF AND EIGTH NOTES !!!)
ssb_bpm = (94/1.25) # slow down slightly cause it sounds better to me
ssb_notes = [
    # Measure 1
    Note(ssb_bpm, ("G", 3), True),
    Note(ssb_bpm, ("E", 3)),
    Note(ssb_bpm, ("C", 3)),
    Note(ssb_bpm, ("E", 3)),
    Note(ssb_bpm, ("G", 3)),
    Note(ssb_bpm, ("C", 4), True),
    Note(ssb_bpm, ("C", 4)),
    Note(ssb_bpm, ("R", 0)),


    # Measure 2
    Note(ssb_bpm, ("E",  4)),
    Note(ssb_bpm, ("D",  4)),
    Note(ssb_bpm, ("C",  4)),
    Note(ssb_bpm, ("E",  3), True),
    Note(ssb_bpm, ("F#", 3)),
    Note(ssb_bpm, ("G",  3), True),
    Note(ssb_bpm, ("G",  3)),
    Note(ssb_bpm, ("R",  0)),


    # Measure 3
    Note(ssb_bpm, ("G", 3)),
    Note(ssb_bpm, ("G", 3)),
    Note(ssb_bpm, ("E", 4), True),
    Note(ssb_bpm, ("D", 4)),
    Note(ssb_bpm, ("C", 4)),
    Note(ssb_bpm, ("B", 3), True),
    Note(ssb_bpm, ("B", 3)),
    Note(ssb_bpm, ("R", 0)),


    # Measure 4
    Note(ssb_bpm, ("A", 3)),
    Note(ssb_bpm, ("B", 3)),
    Note(ssb_bpm, ("C", 4), True),
    Note(ssb_bpm, ("C", 4)),
    Note(ssb_bpm, ("G", 3)),
    Note(ssb_bpm, ("E", 3), True),
    Note(ssb_bpm, ("C", 3), True),
    Note(ssb_bpm, ("C", 3)),
    Note(ssb_bpm, ("R", 0)),

    # Measure 5
    Note(ssb_bpm, ("E", 3)),
    Note(ssb_bpm, ("G", 3)),
    Note(ssb_bpm, ("C", 3)),
    Note(ssb_bpm, ("E", 3)),
    Note(ssb_bpm, ("G", 3)),
    Note(ssb_bpm, ("C", 4), True),
    Note(ssb_bpm, ("C", 4)),
    Note(ssb_bpm, ("R", 0)),

    # Measure 6
    Note(ssb_bpm, ("E",  4)),
    Note(ssb_bpm, ("D",  4)),
    Note(ssb_bpm, ("C",  4), True),
    Note(ssb_bpm, ("E",  3), True),
    Note(ssb_bpm, ("F#", 3)),
    Note(ssb_bpm, ("G",  3), True),
    Note(ssb_bpm, ("G",  3)),
    Note(ssb_bpm, ("R",  0)),

    # Measure 7
    Note(ssb_bpm, ("G",  3)),
    Note(ssb_bpm, ("G",  3)),
    Note(ssb_bpm, ("E",  4), True),
    Note(ssb_bpm, ("D",  4)),
    Note(ssb_bpm, ("C",  4)),
    Note(ssb_bpm, ("B",  3), True),
    Note(ssb_bpm, ("B",  3)),
    Note(ssb_bpm, ("R",  0)),

    # Measure 8
    Note(ssb_bpm, ("A",  3)),
    Note(ssb_bpm, ("B",  3)),
    Note(ssb_bpm, ("C",  4), True),
    Note(ssb_bpm, ("C",  4), True),
    Note(ssb_bpm, ("G",  3)),
    Note(ssb_bpm, ("E",  3), True),
    Note(ssb_bpm, ("C",  3), True),
    Note(ssb_bpm, ("C",  3)),
    Note(ssb_bpm, ("R",  0)),


    # Measure 9
    Note(ssb_bpm, ("E",  4)),
    Note(ssb_bpm, ("E",  4)),
    Note(ssb_bpm, ("E",  4), True),
    Note(ssb_bpm, ("F",  4)),
    Note(ssb_bpm, ("G",  4)),
    Note(ssb_bpm, ("G",  4), True),
    Note(ssb_bpm, ("G",  4)),
    Note(ssb_bpm, ("R",  0)),

    # Measure 10
    Note(ssb_bpm, ("F",  4)),
    Note(ssb_bpm, ("E",  4)),
    Note(ssb_bpm, ("D",  4), True),
    Note(ssb_bpm, ("E",  4)),
    Note(ssb_bpm, ("F",  4)),
    Note(ssb_bpm, ("F",  4), True),
    Note(ssb_bpm, ("F",  4)),
    Note(ssb_bpm, ("R",  0)),

    # Measure 11
    Note(ssb_bpm, ("F",  4)),
    Note(ssb_bpm, ("E",  4)),
    Note(ssb_bpm, ("D",  4), True),
    Note(ssb_bpm, ("C",  4)),
    Note(ssb_bpm, ("B",  3), True),
    Note(ssb_bpm, ("B",  3)),
    Note(ssb_bpm, ("R",  0)),

    # Measure 12
    Note(ssb_bpm, ("A",  3)),
    Note(ssb_bpm, ("B",  3)),
    Note(ssb_bpm, ("C",  4)),
    Note(ssb_bpm, ("E",  3)),
    Note(ssb_bpm, ("F#", 3)),
    Note(ssb_bpm, ("G",  3), True),
    Note(ssb_bpm, ("G",  3)),
    Note(ssb_bpm, ("R",  0)),

    # Measure 13
    Note(ssb_bpm, ("G",  3)),
    Note(ssb_bpm, ("C",  4)),
    Note(ssb_bpm, ("C",  4)),
    Note(ssb_bpm, ("C",  4), True),
    Note(ssb_bpm, ("B",  3)),
    Note(ssb_bpm, ("A",  3)),
    Note(ssb_bpm, ("A",  3)),
    Note(ssb_bpm, ("A",  3)),
    Note(ssb_bpm, ("D",  4), True),
    Note(ssb_bpm, ("F",  4)),
    Note(ssb_bpm, ("E",  4)),
    Note(ssb_bpm, ("D",  4), True),
    Note(ssb_bpm, ("E",  4), True),
    Note(ssb_bpm, ("C",  4), True),
    Note(ssb_bpm, ("B",  3), True),
    Note(ssb_bpm, ("B",  3)),
    Note(ssb_bpm, ("R",  0)),

    # Measure 14
    Note(ssb_bpm, ("G",  3)),
    Note(ssb_bpm, ("G",  3)),
    Note(ssb_bpm, ("C",  4), True),
    Note(ssb_bpm, ("D",  4)),
    Note(ssb_bpm, ("E",  4)),
    Note(ssb_bpm, ("F",  4)),
    Note(ssb_bpm, ("G",  4), True),
    Note(ssb_bpm, ("G",  4)),
    Note(ssb_bpm, ("R",  0)),

    # Measure 15
    Note(ssb_bpm, ("C",  4)),
    Note(ssb_bpm, ("E",  4)),
    Note(ssb_bpm, ("E",  4), True),
    Note(ssb_bpm, ("F",  4)),
    Note(ssb_bpm, ("D",  4)),
    Note(ssb_bpm, ("C",  4), True),
    Note(ssb_bpm, ("C",  4), True),
    Note(ssb_bpm, ("C",  4)),
    Note(ssb_bpm, ("R",  0)),

    # Extra rests at the end for cleaner looping
    Note(ssb_bpm, ("R",  0)),
    Note(ssb_bpm, ("R",  0)),
    Note(ssb_bpm, ("R",  0)),
    Note(ssb_bpm, ("R",  0)),
    Note(ssb_bpm, ("R",  0)),
    Note(ssb_bpm, ("R",  0)),
    Note(ssb_bpm, ("R",  0)),
    Note(ssb_bpm, ("R",  0)),
]

# define Not Like Us notes
nlu_bpm = 101
nlu_intro_notes = [
    # intro part, measure 1
    Note(nlu_bpm,   ("R",  5)),
    Note(nlu_bpm,   ("R",  5)),
    Note(nlu_bpm,   ("R",  5)),
    Note(nlu_bpm,   ("E",  4)),
    Note(nlu_bpm,   ("C#", 4)), # actually is D flat
    Note(nlu_bpm,   ("E",  4), True),
    Note(nlu_bpm/2, ("E",  4)), # bpm/2 should be 8th notes
    Note(nlu_bpm,   ("R",  4)),
    Note(nlu_bpm/2, ("D",  4)), 
    Note(nlu_bpm,   ("B",  3)), 
    
    # intro part, measure 2
    Note(nlu_bpm,   ("D",  4)),
    Note(nlu_bpm,   ("B",  3)), 
    Note(nlu_bpm,   ("F#", 4)),
    Note(nlu_bpm/2, ("E",  4)), 
    Note(nlu_bpm/2, ("D",  4)), 
]

nlu_notes = [
    # main melody loop
    Note(nlu_bpm,   ("F#", 4)),
    Note(nlu_bpm,   ("F#", 4)),
    Note(nlu_bpm,   ("G",  4)),
    Note(nlu_bpm,   ("F#", 4)),
    Note(nlu_bpm,   ("R",  4)),
    Note(nlu_bpm,   ("R",  4)),
    Note(nlu_bpm,   ("R",  4)),

    # main melody loop pt 2
    Note(nlu_bpm,   ("F#", 4)),
    Note(nlu_bpm,   ("F#", 4)),
    Note(nlu_bpm,   ("G",  4)),
    Note(nlu_bpm,   ("F#", 4), True),
    Note(nlu_bpm,   ("F#", 4), True),
    Note(nlu_bpm/2, ("F#", 4)),
    Note(nlu_bpm/2, ("R",  4)),

    # rest for good spacing
    Note(nlu_bpm,   ("R",  5)),
]

class SongNode(Node):
    def __init__(self, name, ssb_toggle = False, time_sleep = 2):
        self.ssb_toggle = ssb_toggle

        # init node
        super().__init__(name)
        # init buzzer publisher
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)

        # init motor publisher (dancing! :D)
        self.motor_pub = self.create_publisher(MotorsState, 'ros_robot_controller/set_motor', 1)
        # init mecanum chassis controller object
        self.mecanum = MecanumChassis()    

        # setup buzzer msg to be sent
        self.buzzer_msg = BuzzerState()
        self.buzzer_msg.repeat = 1

        # time sleep for consistency
        time.sleep(time_sleep)


    def set_velocity(self, linear_x, linear_y, angular_z):
        speeds = self.mecanum.set_velocity(linear_x, linear_y, angular_z)
        self.motor_pub.publish(speeds)

    def sing(self):
        # loop through each note and play it continuously
        intro_done = False

        # dance_moves = [
        #     (.25, .25, 0),
        #     (0, 0, -2),
        #     (0, 0, -2),
        #     (.25, 0, 0),
        #     (0, 0, -2),
        #     (0, 0, -2),
        #     (0, 0, -2),
        #     (.25, 0, 0),
        #     (0, 0, 2),
        #     (0, 0, 2),
        #     (0, 0, 2),
        #     (.25, 0, 0),
        #     (0, 0, 2),
        #     (0, 0, 2),
        #     (.25, .25, 0),
        # ]

        dance_moves = [
            (0, .25, 1),
            (0, -.25, 1)
        ]

        # dance_moves = [
        #     (0, 0, 0)
        # ]

        while True:
            # pick the correct set of notes to use
            # if not intro_done and not self.ssb_toggle:
            if not intro_done:
                note_objs = nlu_intro_notes
                intro_done = True

            # elif intro_done and not self.ssb_toggle:
            else:
                note_objs = nlu_notes
            
            # else: # only here if ssb_toggle is True
            #     note_objs = ssb_notes

            # sometimes note_obj.off_time == 0, so put a constant value that is not zero here
                # note_obj.off_time == 0 when slur == True
            normal_off_time = note_objs[1].off_time

            dance_idx = 0

            dance_coef = 1            
            # play the notes
            for note_obj in note_objs:
                off_time = note_obj.off_time
                on_time = note_obj.on_time 
                
                # this needs to be here because self.buzzer_msg.off_time doesn't work how I think it should
                time.sleep(normal_off_time)

                # set up the buzzer message
                self.buzzer_msg.on_time = on_time
                self.buzzer_msg.off_time = off_time
                self.buzzer_msg.freq = note_obj.freq
                self.get_logger().info(f"Buzzing at {self.buzzer_msg.freq}")
                self.buzzer_pub.publish(self.buzzer_msg)


                if note_obj.freq != 0:
                    self.set_velocity(dance_moves[dance_idx][0],
                                    dance_moves[dance_idx][1],
                                    dance_coef*dance_moves[dance_idx][2])
                    
                    dance_idx += 1
                    dance_idx %= len(dance_moves)
                else:
                    self.set_velocity(0, 0, 0)





# Grabbed straight from their code
class MecanumChassis:
    # wheelbase = 0.1368   # 前后轴距(distance between front and real axles)
    # track_width = 0.1446 # 左右轴距(distance between left and right axles)
    # wheel_diameter = 0.065  # 轮子直径(wheel diameter)
    def __init__(self, wheelbase=0.1368, track_width=0.1410, wheel_diameter=0.065):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    def speed_covert(self, speed):
        """
        covert speed m/s to rps/s
        :param speed:
        :return:
        """
        # distance / circumference = rotations per second
        return speed / (math.pi * self.wheel_diameter)

    def set_velocity(self, linear_x, linear_y, angular_z):
        """
        Use polar coordinates to control moving
                    x
        v1 motor1|  ↑  |motor3 v3
          +  y - |     |
        v2 motor2|     |motor4 v4
        :param speed: m/s
        :param direction: Moving direction 0~2pi, 1/2pi<--- ↑ ---> 3/2pi
        :param angular_rate:  The speed at which the chassis rotates rad/sec
        :param fake:
        :return:
        """
        # vx = speed * math.sin(direction)
        # vy = speed * math.cos(direction)
        # vp = angular_rate * (self.wheelbase + self.track_width) / 2
        # v1 = vx - vy - vp
        # v2 = vx + vy - vp
        # v3 = vx + vy + vp
        # v4 = vx - vy + vp
        # v_s = [self.speed_covert(v) for v in [v1, v2, -v3, -v4]]
        motor1 = (linear_x - linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor2 = (linear_x + linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor3 = (linear_x + linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
        motor4 = (linear_x - linear_y + angular_z * (self.wheelbase + self.track_width) / 2)

        v_s = [self.speed_covert(v) for v in [-motor1, -motor2, motor3, motor4]]
        data = []
        for i in range(len(v_s)):
            msg = MotorState()
            msg.id = i + 1
            msg.rps = float(v_s[i])
            data.append(msg)
        
        msg = MotorsState()
        msg.data = data
        return msg

def sigint_handler(sig, frame):
    song = SongNode('Song_Node_Die', time_sleep = 0)
    song.set_velocity(0, 0, 0)
    song.buzzer_msg.on_time = 0.0
    song.buzzer_msg.off_time = 0.0
    song.buzzer_msg.freq = 0
    song.buzzer_pub.publish(song.buzzer_msg)
    sys.exit(0)

def main(args=None):
    # init everything
    rclpy.init(args=args)
    
    signal.signal(signal.SIGINT, sigint_handler)

    song = SongNode('Song_Node')
    time.sleep(2)

    # song.set_velocity(0, 0, 0)
    # this enters a while (True) loop
    song.sing()


    # shut down our node and buzzer if we hit here; would need this to be in a SIGINT handler to do anything
    song.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
