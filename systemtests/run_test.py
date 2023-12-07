#!/usr/bin/env python3   
#!/usr/bin/env python3   
from subprocess import Popen, PIPE, TimeoutExpired
import time
import os
import signal
from mcap_handler import McapHandler
from datetime import datetime
from plotter_class import Plotter
from pathlib import Path
import shutil
import atexit


def clean_process(process:Popen) -> int :
    '''Kills process and its children on exit if they aren't already terminated (called with atexit). Returns 0 on success, 1 on failure''' 

    if process.poll() == None:
        group_id = os.getpgid(process.pid)
        print(f"cleaning process {group_id}")
        os.killpg(group_id, signal.SIGTERM)
        time.sleep(0.01) #necessary delay before first poll
        i=0
        while i < 10 and process.poll() == None:  #in case the process termination is lazy and takes some time, we wait up to 0.5 sec per process
            if process.poll() != None:
                return 0  #if we have a returncode-> it terminated
            time.sleep(0.05) #if not wait a bit longer
        if(i == 9):
            print(f"Process group {process} with groupID {group_id} didn't terminate correctly")
            return 1  #after 0.5s we stop waiting and consider it did not terminate correctly
        return 0
    else:
        return 0 #process already terminated


def record_start_and_clean(testname:str, max_wait:int, bagfolder:str):
    '''Starts recording the /tf topic in a rosbag, starts the test, waits, closes the rosbag and terminate all processes. max_wait is the max amount of time you want to wait 
        before forcefully terminating the test flight script (in case it never finishes correctly).
        NB the testname must be the name of the crayzflie_examples executable (ie the CLI grammar "ros2 run crazyflie_examples testname" must be valid)'''
    
    index = bagfolder.find("bag_")
    bagname = bagfolder[index:]
    src = "source " + bagfolder[:index-9] + "install/setup.bash" # -> "source /home/github/actions-runner/_work/crazyswarm2/crazyswarm2/ros2_ws/install/setup.bash"

    try:
        command = f"{src} && ros2 bag record -s mcap -o {testname}_{bagname} /tf"
        record_bag =  Popen(command, shell=True, cwd=bagfolder, stderr=PIPE, stdout=True, 
                            start_new_session=True, text=True, executable="/bin/bash") 
        atexit.register(clean_process, record_bag)

        command = f"{src} && ros2 run crazyflie_examples {testname}"
        start_flight_test = Popen(command, shell=True, stderr=True, stdout=True, 
                                start_new_session=True, text=True, executable="/bin/bash")
        atexit.register(clean_process, start_flight_test)

        start_flight_test.wait(timeout=max_wait)  #raise Timeoutexpired after max_wait seconds if start_flight_test didn't finish by itself
        clean_process(start_flight_test)          
        clean_process(record_bag)

    except TimeoutExpired:      #if max_wait is exceeded
        clean_process(start_flight_test)          
        clean_process(record_bag)

    except KeyboardInterrupt:   #if drone crashes, user can ^C to skip the waiting
        clean_process(start_flight_test)          
        clean_process(record_bag)

    #if something went wrong with the bash command lines in Popen, print the error
    if record_bag.stderr != None:
        print(testname," record_bag stderr: ", record_bag.stderr.readlines())
    if start_flight_test.stderr != None:
        print(testname," start_flight flight stderr: ", start_flight_test.stderr.readlines())


def translate_and_plot(testname:str, bagfolder:str):
    '''Translates rosbag .mcap format to .csv, then uses that csv to plot a pdf '''

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

    path = Path(__file__)           #Path(__file__) in this case "/home/github/actions-runner/_work/crazyswarm2/crazyswarm2/ros2_ws/src/crazyswarm2/systemtests/newsub.py" ; path.parents[0]=.../systemstests
    
    #delete results, logs and bags of previous experiments if they exist
    if(Path(path.parents[3].joinpath("bagfiles")).exists()):
        shutil.rmtree(path.parents[3].joinpath("bagfiles"))
    if(Path(path.parents[3].joinpath("results")).exists()):
        shutil.rmtree(path.parents[3].joinpath("results"))  
    if(Path(Path.home() / ".ros/log").exists()):
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
    atexit.register(clean_process, launch_crazyswarm)  #atexit helps us to make sure processes are cleaned even if script exits unexpectedly
     
    time.sleep(1)
    record_start_and_clean("figure8", 20, bagfolder)
    record_start_and_clean("multi_trajectory", 80, bagfolder)

    clean_process(launch_crazyswarm)   #kill crazyswarm and all of its child processes


    #test done, now we create the results pdf 
    translate_and_plot("figure8", bagfolder)
    translate_and_plot("multi_trajectory", bagfolder)

    exit(0)
