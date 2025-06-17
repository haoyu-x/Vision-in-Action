import os
import time
import argparse
import sys, tty, termios
import subprocess
import signal
import rosbag
import rospy
import pyttsx3

# Initialize the text-to-speech engine

engine = pyttsx3.init()
good_counter = 0

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output_path", default="/home/haoyux/teddy_data", help="path to directory to output bag")

    args = parser.parse_args()
    return args

def get_bag_path(output_path: str):
    f = time.strftime("%m_%d_%y-%H:%M:%S.bag")

    return os.path.join(output_path, f)


def record_command(args):
    output_path = get_bag_path(args.output_path)

    if os.path.isdir(args.output_path):
        print(f"directory {args.output_path} already exists....logging")
    else:
        os.makedirs(args.output_path)



        

    all_topics = [
            # iphone
            "/camera/iphone_rgb",
            "/camera/iphone_depth",
            "/camera/iphone_extrinsic",
            "/camera/iphone_intrinsic",
            
            # exo
            "/exo/iphone_rgb",
            "/exo/iphone_depth",
            "/exo/iphone_extrinsic",
            "/exo/iphone_intrinsic",

            # webcam 
            "/camera/right_RGB",
            "/camera/left_RGB",
            "/camera/top_RGB",
      
            # neck
            "/neck/proprio",
            "/neck/neck_redundant",
            "/neck/action",

            # arms
            "/arm/right_proprio",
            "/arm/right_redundant",

            "/arm/left_proprio",
            "/arm/left_redundant",

            # gello
            "/arm/right_gello",
            "/arm/left_gello",
            ]

    # check_missing_topics(all_topics)
    # all_nodes = " ".join(nodes)
    # command = f"rosbag record {all_nodes} -O {output_path}"
    # topic = "/camera/RGB"

    all_topics = " ".join(all_topics)

    command = f"rosbag record {all_topics} -O {output_path}"


    return command, output_path

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return ch

def read_it(input_sentence):

    # Initialize the text-to-speech engine
    engine = pyttsx3.init()

    # Input sentence to be read aloud

    # Set properties (optional)
    engine.setProperty('rate', 100)  # Speed of speech
    engine.setProperty('volume', 2)  # Volume level (0.0 to 1.0)

    # Make the engine speak the input sentence
    engine.say(input_sentence)

    # Run and wait for the speech to complete
    engine.runAndWait()




def record(args, prev_path=None, p=None):
    global good_counter
    global old_time
    
    print("Type s to start, k to kill and end, g to restart and keep good bag, b to restart and delete bad bag.")
    # read_it("Type s to start, k to kill and end, g to restart and keep good bag, b to restart and delete bad bag.")
    cmd = getch() # str(input()).strip()
    p_new = None
    fpath = None
    if cmd == 's':
        # old_time = time.time()
        command, fpath = record_command(args)
        p_new = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        #print(f"started process {p_new.pid}")
        read_it("started")

    elif cmd == 'k' or cmd == 'x':
        if p is not None:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
            print(f"killed process {p.pid}")
            read_it("process killed")

            time.sleep(2)
            os.remove(prev_path)#+'.active')
            print(f"removed file at location {prev_path}")
            read_it("file removed")

            return
        else:
            print("invalid. No process is running")
            read_it("invalid. No process is running. exit")

            exit()
    elif cmd == 'g' or cmd == 'b':
        if p is not None:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
            print(f"killed process {p.pid}")

            time.sleep(1.5)
            bag = rosbag.Bag(prev_path)
            if bag.get_message_count() < 10:
                print("Message count" + str(bag.get_message_count()))
                print('BAG IS EMPTY!!!!!!!!!!')
                print('BAG IS EMPTY!!!!!!!!!!')
                print('BAG IS EMPTY!!!!!!!!!!')
                read_it('BAG IS EMPTY')

            else:
                print("Message count" + str(bag.get_message_count()))
            bag.close()
        if cmd == 'b':
            os.remove(prev_path)#+'.active')
            print(f"removed file at location {prev_path}")
            read_it('Bad demo, removed')

        else:
            good_counter += 1
            print(f'reset: {good_counter} good trajectories saved')
            read_it(f'{good_counter} demo saved')

        # print(f'Time: {time.time() - old_time} seconds')
        # old_time = time.time()
        # command, fpath = record_command(args)
        # p_new = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        #print(f"started process {p_new.pid}")

    else:
        print("invalid character")

    record(args, fpath, p_new)
    if p is not None:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)

def check_missing_topics(desired_topics):
    # Get all currently available topics
    available_topics = [topic for topic, _ in rospy.get_published_topics()]
    
    # Identify any desired topics that are missing
    missing_topics = [topic for topic in desired_topics if topic not in available_topics]
    
    if missing_topics:
        print("Error: Missing topics:", missing_topics)
        print("Terminating program due to missing topics.")
        sys.exit(1)  # Exit the program if any topics are missing
    else:
        print("All desired topics are available. Continuing with the program.")


if __name__ == '__main__':
    args = get_args()

    record(args)