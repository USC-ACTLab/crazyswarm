#!/usr/bin/env python3   
#!/usr/bin/env python3   
from subprocess import Popen, PIPE
import time
import os
import signal
from mcap_handler import McapHandler
from datetime import datetime
from plotter_class import Plotter
from pathlib import Path
import shutil
import atexit


class Waiter:
    '''A helper class which sleeps the amount of seconds it is instanciated with. If the user interrupts with ^C during the waiting time (zB if the drone crashes), it will stop sleeping and continue
        to the end of the program where all child processes are terminated correctly. All subsequent Waiter() calls will be ignored'''
    wait=True

    def __init__(self, seconds):
        if seconds > 0 :
            self.sleep(seconds)
    
    def sleep(self, seconds):
        start=time.time()
        while Waiter.wait and time.time()<(start+seconds):
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                Waiter.wait = False

def record_start_and_terminate(testname:str, testduration:int, bagfolder:str):
    '''Helper function that starts recording the /tf topic in a rosbag, starts the test, waits, closes the rosbag and terminate all processes
        NB the testname must be the name of the crayzflie_examples executable (ie the CLI grammar "ros2 run crazyflie_examples testname" must be valid)'''
    index = bagfolder.find("bag_")
    bagname = bagfolder[index:]

    
    src = "source " + bagfolder[:index-9] + "install/setup.bash" # -> "source /home/github/actions-runner/_work/crazyswarm2/crazyswarm2/ros2_ws/install/setup.bash"
    
    command = f"{src} && ros2 bag record -s mcap -o {testname}_{bagname} /tf"
    record_bag =  Popen(command, shell=True, cwd=bagfolder, stderr=PIPE, stdout=True, 
                        start_new_session=True, text=True, executable="/bin/bash") 

    command = f"{src} && ros2 run crazyflie_examples {testname}"
    start_flight_test = Popen(command, shell=True, stderr=True, stdout=True, 
                              start_new_session=True, text=True, executable="/bin/bash")
    

    Waiter(testduration)  #wait x seconds for the crazyflie to fly the test

    os.killpg(os.getpgid(start_flight_test.pid), signal.SIGTERM)  #kill flight test and all of its child processes
    os.killpg(os.getpgid(record_bag.pid), signal.SIGTERM) #kill rosbag 

    #if something went wrong with the bash command lines in Popen, print the error
    if record_bag.stderr != None:
        print(testname," record_bag stderr: ", record_bag.stderr.readlines())
    if start_flight_test.stderr != None:
        print(testname," start_flight flight stderr: ", start_flight_test.stderr.readlines())



def translate_and_plot(testname:str, bagfolder:str):
    '''Helper function that translates rosbag .mcap format to .csv, then uses that csv to plot a pdf '''
    index = bagfolder.find("bag_")
    bagname = bagfolder[index:]
    # NB : the mcap filename is almost the same as the folder name but has _0 at the end
    inputbag = str(bagfolder) + f"/{testname}_{bagname}/{testname}_{bagname}_0.mcap"
    output_csv = str(bagfolder) + f"/{testname}_{bagname}/{testname}_{bagname}_0.csv"
    writer = McapHandler()
    writer.write_mcap_to_csv(inputbag, output_csv)  #translate bag from mcap to csv
    output_pdf = str(path.parents[3].joinpath(f"results/Results_{testname}_"+ now +".pdf"))
    rosbag_csv = output_csv
    if "figure8" in testname:
        test_file = "../crazyflie_examples/crazyflie_examples/data/figure8.csv"
    elif "multi_trajectory" in testname:
        test_file = "../crazyflie_examples/crazyflie_examples/data/multi_trajectory/traj0.csv"
    else:
        print("run_test.py : test file not defined")
    plotter = Plotter()
    plotter.create_figures(test_file, rosbag_csv, output_pdf) #plot the data


if __name__ == "__main__":

    path = Path(__file__)           #Path(__file__) should be "/home/github/actions-runner/_work/crazyswarm2/crazyswarm2/ros2_ws/src/crazyswarm2/systemtests/newsub.py" ; path.parents[0]=.../systemstests
    
    #delete results, logs and bags of previous experiments
    shutil.rmtree(path.parents[3].joinpath("bagfiles"))
    shutil.rmtree(path.parents[3].joinpath("results"))
    shutil.rmtree(Path.home() / ".ros/log")


    #create the folder where we will record the different bags and the folder where the results pdf will be saved
    now = datetime.now().strftime('%d_%m_%Y-%H_%M_%S')
    bagfolder = str((path.parents[3].joinpath(f"bagfiles/bag_" + now)))  # /home/github/actions-runner/_work/crazyswarm2/crazyswarm2/ros2_ws/bagfiles/bag_d_m_Y-H_M_S
    os.makedirs(bagfolder) 
    os.makedirs(path.parents[3].joinpath("results"))  # /home/github/actions-runner/_work/crazyswarm2/crazyswarm2/ros2_ws/results
    
   
    src = "source " + str(path.parents[3].joinpath("install/setup.bash"))  # -> "source /home/github/actions-runner/_work/crazyswarm2/crazyswarm2/ros2_ws/install/setup.bash"
    command = f"{src} && ros2 launch crazyflie launch.py"
    launch_crazyswarm = Popen(command, shell=True, stderr=True, stdout=True, text=True,
                              start_new_session=True, executable="/bin/bash") 
     
    time.sleep(1)
    print("f8")
    record_start_and_terminate("figure8", 20, bagfolder)
    print("multi")
    record_start_and_terminate("multi_trajectory", 80, bagfolder)

    os.killpg(os.getpgid(launch_crazyswarm.pid), signal.SIGTERM)   #kill crazyswarm and all of its child processes


    #test done, now we create the results pdf 
    translate_and_plot("figure8", bagfolder)
    translate_and_plot("multi_trajectory", bagfolder)

    exit(0)
