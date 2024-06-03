import serial  # install pyserial
import serial.tools.list_ports
import datetime
import time
import os
import pandas as pd
import threading

# Created by Kevin Kleczka May 2024, contact kevin.kleczka@northwestern.edu

# Serial communication baud rate, make sure this matches Arduino BAUD rate
BAUD_RATE = 921600
STEPS_PER_REVOLUTION = 400
ALL_FLAG = 999

KEY_LIST = ['timestamp', 'whisker_id', 'stepper_id', 'whisking_status', 'curr_steps', 'sensor_out']

save_interval = 60  # automatically save log every 60 seconds


class MotorController(object):
    def __init__(self, project_dir=None, title_string=None, exp_tag=None, shield_addrs=None, contact_thresh=100,
                 motor_startup='12', num_whiskers_per_col=2, prot_steps=133, ret_steps=22, sampling_period=20000,
                 push_steps=22, pull_steps=6, palpate_num=3, contact_behavior='palpate'):
        """
        Create motor controller object. Searches for an Arduino connected to a usb port and initializes a serial
        connection. If project_dir is not defined, it will create a directory to store the data log file in the
        current working directory named "yourExpTag_currentDateTime". Sets up a thread to read datastream from
        Arduino. Once streaming has started, the data will be automatically saved every 60 seconds by default. This
        interval can be changed by modifying the save_interval variable in this file. Use MotorController.write_log()
        to manually save the log file. All data will be saved to log dataframe. Additionally, lists containing the
        position of each whisker (in steps) and the voltage of each whisker will be stored in the whisker_steps and
        whisker_vals variables for easy access with format motorController.whisker_vals[whisker_id][sample_idx]


        :param project_dir: Directory to store the experiment data
        :param title_string:    Name of the data log file. Recommended to leave this as None. If not defined, the default
        title string of "expTag_currDateTime" will be used
        :param exp_tag: Name of the experiment. If not defined, will be set to "noExpTag"
        :param shield_addrs:    A list of Adafruit Motorshield addresses that are connected to the Arduino. This should
        be the value of the address bits soldered on the shield converted to an integer. For example, if no addr bits
        are soldered, the vale is 0, if 0b0010 are soldered, the value is 2.
        :param contact_thresh:  The number of whisker signal steps above resting value that constitutes a contact
        :param motor_startup:   A string or list of strings that define which steppers will be used on each shield,
        the order they will be referenced, and the side they belong to. Defaults to right side if sides are not defined.
        For example, '12' will initialize steppers 1 and 2 on the right side and reference stepper 1 first. '2' will
        initialize only motor 2 on the right side. '21rl' will initialize stepper 2 on the right side and stepper 1 on
        the left side, and stepper 2 will be referenced first.
        :param num_whiskers_per_col:    Number of whiskers on each stepper.
        :param prot_steps:  The number of steps from the home position to which the columns will protract during a whisk
        :param ret_steps:   The number of steps from the home position to which the columns will retract during a whisk
        :param sampling_period: The number of microseconds between samples. Testing was done at 50 Hz (20000 us)
        :param push_steps:  How many steps to push past the detected contact point when palpating
        :param pull_steps:  Number of steps before contact location to retract to when palpating
        :param palpate_num: Number of times to palpate after a contact has been detected
        :param contact_behavior:  Set to 'palpate' to palpate when a contact is detected. Nothing else has been
        implemented, so setting it to anything else will just ignore contacts
        """

        self.num_shields = 0
        self.num_steppers = 0
        self.num_whiskers = 0
        self.date_time_str = get_datetime_str()
        self.whisking_status = []
        self.whisker_steps = []
        self.whisker_vals = []
        self.last_vals = []
        self.last_steps = []
        self.default_prot_steps = prot_steps
        self.default_ret_steps = ret_steps
        self.prot_steps = []
        self.ret_steps = []
        self.sampling = 0
        self.in_whisk = 0
        self.whisking_stopped = threading.Event()
        self.whisking_stopped.clear()
        self.last_save = time.time()
        self.sampling_period = sampling_period
        self.data_ready = threading.Event()

        self.is_whisking = 0
        self.whisk_commanded = 0

        self.contact_behavior = contact_behavior
        self.contact_thresh = contact_thresh
        self.push_steps = push_steps
        self.pull_steps = pull_steps
        self.palpate_num = palpate_num
        self.thresh_arr = []
        self.is_contact = []
        self.is_palpating = 0

        if not title_string:
            if not exp_tag:
                exp_tag = 'noExpTag'
            title_string = exp_tag + '_' + self.date_time_str
        self.title_string = title_string

        if not project_dir:
            project_dir = self.title_string
        if not os.path.isdir(project_dir):
            os.mkdir(project_dir)
        self.project_dir = project_dir

        self.log = pd.DataFrame(columns=KEY_LIST)

        # connect to Arduino
        ports = list(serial.tools.list_ports.comports())  # Create a list with info for all devices
        connected = False
        # Look for the word 'Arduino' in device info, store this port
        for p in ports:
            if "Arduino" in p[1]:
                connected = True
                port_arduino = p[0]
                break
        if not connected:
            print("No Arduino Found")
        else:
            self.ser = serial.Serial(port_arduino, BAUD_RATE)  # Create serial object for the Arduino
            print('Arduino connected. Port: ' + port_arduino)

            while not self.ser.in_waiting:
                pass
            last_char = self.ser.read().decode()
            while last_char != '!':
                try:
                    last_char = self.ser.read().decode()
                except:
                    pass
            time.sleep(.25)
            while self.ser.in_waiting:
                self.ser.read()
            print('Handshake complete')

        self.shield_addrs = shield_addrs
        self.motor_startup = motor_startup
        if not self.shield_addrs:
            print("No project settings. Make sure to add shields before running init() or create a new motor "
                  "controller with shield addresses.")
        else:
            if type(shield_addrs) is not list:
                self.shield_addrs = [self.shield_addrs]
            for i in range(len(self.shield_addrs)):
                if type(self.motor_startup) is list:
                    ms = self.motor_startup[i]
                else:
                    ms = self.motor_startup
                self.add_shield(self.shield_addrs[i], ms, num_whiskers_per_col)

        if sampling_period:
            self.send_cmd('p ' + str(sampling_period))

        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._read_sample)
        self.thread.daemon = True  # Ensure the thread exits when the main program exits

    def init(self):
        """
        Initializes the experiment. Completes setup, homes steppers, and starts sampling
        :return:
        """

        for i in range(self.num_whiskers):
            self.thresh_arr.append(1024)
            self.is_contact.append(0)

        # send command to complete setup
        self.send_cmd('c')

        # initialize prot and ret steps
        if self.default_prot_steps:
            for i in range(self.num_whiskers):
                self.prot_steps.append(self.default_prot_steps)
            self.set_prot(self.default_prot_steps)
        if self.default_ret_steps:
            for i in range(self.num_whiskers):
                self.ret_steps.append(self.default_ret_steps)
            self.set_ret(self.default_ret_steps)

        # home motors
        self.send_cmd('home_stepper ' + str(ALL_FLAG))

        contact_behavior = self.contact_behavior
        self.contact_behavior = None
        self._start_sampling()
        time.sleep(.5)

        # go to retracted position
        self.retract()

        # set thresholds relative to resting values
        self.whisking_stopped.wait()
        self.data_ready.wait()
        time.sleep(2.5)
        for i in range(self.num_whiskers):
            self.thresh_arr[i] = self.whisker_vals[i][-1] + self.contact_thresh
            self.is_contact[i] = 0

        self.contact_behavior = contact_behavior

    def _read_sample(self):
        """
        ISR to read datastream from Arduino. Will set whisking_stopped flag when a whisk completes
        :return:
        """

        last_timestamp = 0
        sample_count = 0
        palpate_count = 0
        while not self.stop_event.is_set():
            if self.ser.in_waiting:

                proc = False
                try:
                    line = self.ser.readline().decode('utf-8').rstrip()
                    proc = True
                except:
                    clr = False
                    while self.ser.in_waiting and not clr:
                        last_char = self.ser.read().decode()
                        if last_char == r'\n':
                            clr = True
                    print("bad line read")
                if proc:
                    parts = line.split(' ')
                    # KEY_LIST = ['timestamp', 'whisker_id', 'stepper_id', 'whisking_status', 'curr_steps', 'sensor_out']
                    if len(parts) == len(KEY_LIST) and all(p.isdigit() for p in parts):
                        new_row = {}
                        for i in range(len(KEY_LIST)):
                            k = KEY_LIST[i]
                            new_row[k] = parts[i]
                        self.log.loc[len(self.log)] = new_row
                        # stepper_id = int(new_row['stepper_id'])
                        whisker_id = int(new_row['whisker_id'])
                        steps = int(new_row['curr_steps'])
                        val = int(new_row['sensor_out'])
                        timestamp = int(new_row['timestamp'])

                        if timestamp == last_timestamp:
                            sample_count += 1
                            self.data_ready.clear()
                        else:
                            sample_count = 0
                        last_timestamp = timestamp
                        if whisker_id < self.num_whiskers:
                            self.whisking_status[whisker_id] = new_row['whisking_status']
                            self.whisker_steps[whisker_id].append(steps)
                            self.whisker_vals[whisker_id].append(val)
                            self.last_steps[whisker_id] = steps
                            self.last_vals[whisker_id] = val
                    else:
                        print("bad line read")

                if sample_count == 0:
                    if any([int(self.whisking_status[i]) for i in range(self.num_whiskers)]):
                        self.in_whisk = 1
                        self.whisking_stopped.clear()
                        self.whisk_commanded = 0
                    else:
                        if not self.whisk_commanded:
                            self.whisking_stopped.set()
                            self.in_whisk = 0
                        else:
                            self.whisk_commanded -= 1

                    if self.is_whisking:
                        if self.contact_behavior == 'palpate':
                            if not self.is_palpating:
                                if any([self.last_vals[i]>self.thresh_arr[i] for i in range(self.num_whiskers)]):
                                    curr_steps = round(sum(self.last_steps) / len(self.last_steps))
                                    target_prot = curr_steps + self.push_steps
                                    target_ret = curr_steps - self.pull_steps
                                    self.set_ret(target_ret)
                                    time.sleep(.001)
                                    self.retract()
                                    time.sleep(.001)
                                    self.set_prot(target_prot)
                                    time.sleep(.001)
                                    self.is_palpating = 1
                                    palpate_count = 0
                                else:
                                    if self.whisking_stopped.is_set():
                                        self.whisk()
                            else:
                                if self.whisking_stopped.is_set():
                                    if palpate_count == self.palpate_num:
                                        self.is_palpating = 0
                                        self.set_ret(self.default_ret_steps)
                                        time.sleep(.001)
                                        self.retract()
                                        self.set_prot(self.default_prot_steps)
                                        time.sleep(.001)

                                    else:
                                        palpate_count += 1
                                        self.whisk()
                        else:
                            if self.whisking_stopped.is_set():
                                self.whisk()
                self.data_ready.set()
            if time.time() - self.last_save > save_interval:
                self.write_log()

    def start_whisking(self):
        """
        Startup constant whisking
        :return:
        """
        self.is_whisking = 1
        self.whisk()

    def stop_whisking(self):
        """
        End constant whisking
        :return:
        """
        self.is_whisking = 0

    # def halt_whisk(self, stepper_id=ALL_FLAG):
    #     """
    #     Sends command to stop the current whisk taking acceleration into account.
    #     :param stepper_id: ID of stepper, default is all
    #     :return:
    #     """
    #     self.send_cmd('halt_whisk ' + str(stepper_id))

    def whisk(self, stepper_id=ALL_FLAG):
        """
        Initiate a whisk
        :param stepper_id: ID of stepper, default is all
        :return:
        """

        if not self.sampling:
            self._start_sampling()

        self.whisking_stopped.clear()
        # don't allow whisking_stopped flag to be set until 4 samples have been read to ensure Arduino has started
        # whisking
        self.whisk_commanded = 4
        self.send_cmd('whisk ' + str(stepper_id))
        self.whisking_stopped.clear()

    def retract(self, stepper_id=ALL_FLAG):
        """
        Sends command to go to retracted position
        :param stepper_id: ID of stepper, default is all
        :return:
        """
        self.whisking_stopped.clear()
        self.whisk_commanded = 4
        self.send_cmd('retract ' + str(stepper_id))
        self.whisking_stopped.clear()

    def set_accel(self, accel, stepper_id=ALL_FLAG):
        """
        Set the stepper acceleration param
        :param accel: (int) Steps/second/second
        :param stepper_id: ID of stepper, default is all
        :return:
        """
        self.send_cmd('set_accel ' + str(stepper_id) + ' ' + str(accel))

    def set_speed(self, speed, stepper_id=ALL_FLAG):
        """
        Set the stepper maxSpeed param
        :param speed: (int) Max speed in steps/second
        :param stepper_id: ID of stepper, default is all
        :return:
        """
        self.send_cmd('set_accel ' + str(stepper_id) + ' ' + str(speed))

    # def pause_sampling(self):
    #     self.send_cmd('set_sampling 0')
    #     self.sampling = 0

    def end_exp(self):
        """
        Shut down the current experiment and saves data log file. Recommended to restart ipynb kernel before starting
        a new experiment
        :return:
        """
        self.send_cmd('set_sampling 0')

        self.stop_event.set()
        self.thread.join()
        self.ser.close()
        self.write_log()

    def _start_sampling(self):
        """
        Sends command to start datastream and starts sampling thread
        :return:
        """
        self.send_cmd('set_sampling 1')
        self.sampling = 1
        # self.thread.start()
        if not self.thread.is_alive():
            self.thread.start()
        self.stop_event.clear()

    def set_prot(self, prot_steps, stepper_id=ALL_FLAG):
        """
        Sets the target whisk protraction
        :param prot_steps: (int) Steps from home position
        :param stepper_id: ID of stepper, default is all
        :return:
        """
        self.send_cmd('set_prot ' + str(stepper_id) + ' ' + str(prot_steps))
        if stepper_id == ALL_FLAG:
            for i in range(self.num_steppers):
                self.prot_steps[i] = prot_steps
        else:
            self.prot_steps[stepper_id] = prot_steps

    def set_ret(self, ret_steps, stepper_id=ALL_FLAG):
        """
        Sets the target whisk retraction
        :param ret_steps: (int) Steps from home position
        :param stepper_id: ID of stepper, default is all
        :return:
        """
        self.send_cmd('set_ret ' + str(stepper_id) + ' ' + str(ret_steps))
        if stepper_id == ALL_FLAG:
            for i in range(self.num_steppers):
                self.ret_steps[i] = ret_steps
        else:
            self.ret_steps[stepper_id] = ret_steps

    def add_shield(self, shield_addr, ms, num_whiskers):
        """
        Not recommended to call directly. Called during setup of motorController object. Adds an Adafruit Motorshield
        to the motorController and initializes steppers and whiskers associated with the shield.
        :param shield_addr: (int) Value of addr bits set
        :param ms:  motor setup string
        :param num_whiskers:    number of whiskers per stepper
        :return:
        """
        self.num_shields += 1
        motor_nums = []
        motor_sides = []
        for c in ms:
            if c.isdigit():
                motor_nums.append(c)
            else:
                if c == 'l' or c == 'L':
                    motor_sides.append('1')
                else:
                    motor_sides.append('0')

        for i in range(len(motor_nums) - len(motor_sides)):
            if motor_sides:
                motor_sides.append(motor_sides[0])
            else:
                motor_sides.append('0')
        num_whiskers = num_whiskers*len(motor_nums)
        self.num_whiskers += num_whiskers
        self.num_steppers += len(motor_nums)

        for i in range(self.num_whiskers - len(self.whisking_status)):
            self.whisking_status.append(0)
            self.whisker_steps.append([0])
            self.whisker_vals.append([0])
            self.last_steps.append(0)
            self.last_vals.append(0)

        cmd = 's ' + str(shield_addr) + ' '
        for c in motor_nums:
            cmd = cmd + c
        cmd = cmd + ' ' + str(num_whiskers) + ' '
        for c in motor_sides:
            cmd = cmd + c

        self.send_cmd(cmd)

    def send_cmd(self, cmd):
        """
        Send string command to Arduino
        :param cmd: (string) command to send
        :return:
        """
        # print(cmd)
        for c in cmd:
            self.ser.write(c.encode())
            time.sleep(.00001)
        self.ser.write('\r'.encode())
        time.sleep(.01)

    def write_log(self, file_path=None):
        """
        Saves the data log file to .csv file. Unless a specific filepath is specified, saves as
        "expTag_dateTimeMotorControllerObjectCreated.csv" in the project directory
        :param file_path: full path (including file name) to write data
        :return:
        """
        if not file_path:
            file_path = os.path.join(self.project_dir, self.title_string + '.csv')

        if '.csv' not in file_path:
            file_path += '.csv'

        self.log.to_csv(file_path)
        self.last_save = time.time()
        print('log saved to  %s' % file_path)


def get_datetime_str():
    """
    returns current datetime as a string with format yyyy-mm-dd-HH-MM-SS
    :return: returns current datetime as a string with format yyyy-mm-dd-HH-MM-SS
    """
    date_time = datetime.datetime.now()
    time_str = str(date_time.time()).split('.')[0]  # Get string with just hh:mm:ss
    time_str = time_str.replace(':', '-')  # Now hh-mm-ss
    date_str = str(date_time.date())  # Date in format yyyy-mm-dd
    date_time_str = '-'.join([date_str, time_str])
    return date_time_str


def steps_to_deg(steps):
    """
    Converts steps to degrees based on STEPS_PER_REVOLUTION value
    :param steps: steps value to be converted
    :return: degrees
    """
    return (360.0 / STEPS_PER_REVOLUTION) * steps


def deg_to_steps(deg):
    """
    Converts degrees to steps based on STEPS_PER_REVOLUTION value. Rounds to nearest step
    :param deg: degrees value to be converted
    :return: (int) steps
    """
    return round((STEPS_PER_REVOLUTION / 360.0) * deg)
