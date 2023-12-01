import matplotlib.pyplot as plt
import os
import sys 
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
from crazyflie_py.uav_trajectory import Trajectory


class Plotter:

    def __init__(self):
        self.params = {'experiment':'1','trajectory':'figure8.csv','motors':'standard motors(CF)', 'propellers':'standard propellers(CF)'}
        self.bag_times = np.empty([0])
        self.bag_x = np.empty([0])
        self.bag_y = np.empty([0])
        self.bag_z = np.empty([0])
        self.ideal_traj_x = np.empty([0])
        self.ideal_traj_y = np.empty([0])
        self.ideal_traj_z = np.empty([0])
        self.euclidian_dist = np.empty([0])
        self.deviation = [] #list of all indexes where euclidian_distance(ideal - recorded) > EPSILON

        self.EPSILON = 0.05 # euclidian distance in [m] between ideal and recorded trajectory under which the drone has to stay to pass the test
        self.DELAY_CONST_FIG8 = 4.75 #this is the delay constant which I found by adding up all the time.sleep() etc in the figure8.py file. This should be implemented better later
        self.ALTITUDE_CONST_FIG8 = 1.0 #this is the altitude given for the takeoff in figure8.py. I should find a better solution than a symbolic constant ?
        self.ALTITUDE_CONST_MULTITRAJ = 1 #takeoff altitude for traj0 in multi_trajectory.py

    def file_guard(self, pdf_path):
        msg = None
        if os.path.exists(pdf_path):
            msg = input("file already exists, overwrite? [y/n]: ")
            if msg == "n":
                print("exiting...")
                sys.exit(0)
            elif msg == "y":
                print("overwriting...")
                os.remove(pdf_path)
            else:
                print("invalid msg...")
                self.file_guard(pdf_path)
        return
    


    def read_csv_and_set_arrays(self, ideal_csvfile, rosbag_csvfile):
        '''Method that reads the csv data of the ideal test trajectory and of the actual recorded trajectory and initializes the attribute arrays'''

        #check which test we are plotting : figure8 or multi_trajectory
        if("fig8" in rosbag_csvfile):
            fig8 = True
            print("Plotting fig8 test data")
        else:
            fig8 = False
            print("Plotting multi_trajectory test data")

        #get ideal trajectory data
        self.ideal_traj_csv = Trajectory()
        try:
            path_to_ideal_csv = os.path.join(os.path.dirname(os.path.abspath(__file__)),ideal_csvfile)
            self.ideal_traj_csv.loadcsv(path_to_ideal_csv)
        except FileNotFoundError:
            print("Plotter : File not found " + path_to_ideal_csv)
            exit(1)


        #get rosbag data
        rosbag_data = np.loadtxt(rosbag_csvfile, delimiter=",")
        
        self.bag_times = np.array(rosbag_data[:,0])
        self.bag_x = np.array(rosbag_data[:,1])
        self.bag_y = np.array(rosbag_data[:,2])
        self.bag_z = np.array(rosbag_data[:,3])
    
        self.adjust_arrays()
        bag_arrays_size = len(self.bag_times)
        print("number of datapoints in self.bag_*: ",bag_arrays_size)

        #####calculate ideal trajectory points corresponding to the times of recorded points 

        self.ideal_traj_x = np.empty([bag_arrays_size])
        self.ideal_traj_y = np.empty([bag_arrays_size])
        self.ideal_traj_z = np.empty([bag_arrays_size])
        self.euclidian_dist = np.empty([bag_arrays_size])
        

        no_match_in_idealcsv=[]

        for i in range(bag_arrays_size):  
            try:
                pos = self.ideal_traj_csv.eval(self.bag_times[i] - self.DELAY_CONST_FIG8).pos
            except AssertionError: 
                no_match_in_idealcsv.append(i)
                pos = [0,0,0]  #for all recorded datapoints who cannot be matched to a corresponding ideal position we assume the drone is on its ground start position (ie those datapoints are before takeoff or after landing)

               
            self.ideal_traj_x[i], self.ideal_traj_y[i]= pos[0], pos[1]
            if(fig8) :                                                    #special case : in fig8 test no altitude is given in the trajectory polynomials but is stays constant after takeoff in figure8.py
                self.ideal_traj_z[i] = self.ALTITUDE_CONST_FIG8
            else :                                                        #for multi_trajectory test the altitude given in the trajectory polynomials is added to the takeoff altitude
                self.ideal_traj_z[i] = self.ALTITUDE_CONST_MULTITRAJ + pos[2]

            self.euclidian_dist[i] = np.linalg.norm([self.ideal_traj_x[i]-self.bag_x[i], 
                                                self.ideal_traj_y[i]-self.bag_y[i], self.ideal_traj_z[i]-self.bag_z[i]])
            if self.euclidian_dist[i] > self.EPSILON:
                self.deviation.append(i)
            
        self.no_match_warning(no_match_in_idealcsv)


    def no_match_warning(self, unmatched_values:list):
        ''' A method which prints a warning saying how many (if any) recorded datapoints could not be matched to an ideal datapoint'''

        no_match_arr = np.array(unmatched_values)

        if no_match_arr.size == 0:
            return
        
        split_index_arr = []

        for i in range(no_match_arr.size - 1):                    #find indexes which are not consecutive
            if(no_match_arr[i+1] != no_match_arr[i]+1):
                split_index_arr.append(i+1)

        banana_split = np.split(no_match_arr, split_index_arr)     #split array into sub-array of consecutive indexes -> each sub-array is a timerange for which ideal positions weren't able to calculated
        print(f"{len(no_match_arr)} recorded positions weren't able to be matched with a specified ideal position and were given the default (0,0,0) ideal position instead.")
        print("Probable reason : their timestamp is before the start of the ideal trajectory or after its end.")
        if len(banana_split)==2:
            timerange1_start = self.bag_times[banana_split[0][0]]
            timerange1_end= self.bag_times[banana_split[0][1]]
            timerange2_start = self.bag_times[banana_split[1][0]]
            timerange2_end = self.bag_times[banana_split[1][1]]
            print(f"These datapoints correspond to the time ranges [{timerange1_start} , {timerange1_end}] and [{timerange2_start} , {timerange2_end}]")



    def adjust_arrays(self):
        ''' Method that trims the self.bag_* attributes to get rid of the datapoints where the drone is immobile on the ground and makes self.bag_times start at 0 [s]'''

        print(f"rosbag initial length {(self.bag_times[-1]-self.bag_times[0]) * 10**-9}s")

        #find the takeoff time and landing times
        ground_level = self.bag_z[0]
        airborne = False
        takeoff_index = None
        landing_index = None
        i=0
        for z_coord in self.bag_z:
            if not(airborne) and z_coord > ground_level + ground_level*(0.1): #when altitude of the drone is 10% higher than the ground level, it started takeoff
                takeoff_index = i
                takeoff_time = self.bag_times[takeoff_index]
                airborne = True
                print(f"takeof time is {(takeoff_time-self.bag_times[0]) * 10**-9}")
            if airborne and z_coord < ground_level + ground_level*(0.1): #find when it lands again
                landing_index = i
                landing_time = self.bag_times[landing_index]
                print(f"landing time is {(landing_time-self.bag_times[0]) * 10**-9}")
                break
            i+=1

        assert (takeoff_index != None) and (landing_index != None), "Plotter : couldn't find drone takeoff or landing"


        ####get rid of datapoints before takeoff and after landing in bag_times, bag_x, bag_y, bag_y   

        assert len(self.bag_times) == len(self.bag_x) == len(self.bag_y) == len(self.bag_z), "Plotter : self.bag_* aren't the same size before trimming"
        index_arr = np.arange(len(self.bag_times))
        slicing_arr = np.delete(index_arr, index_arr[takeoff_index:landing_index+1])  #in our slicing array we take out all the indexes of when the drone is in flight so that it only contains the indexes of when the drone is on the ground

        #delete the datapoints where drone is on the ground
        self.bag_times = np.delete(self.bag_times, slicing_arr)
        self.bag_x = np.delete(self.bag_x, slicing_arr)
        self.bag_y = np.delete(self.bag_y, slicing_arr)
        self.bag_z = np.delete(self.bag_z, slicing_arr)

        assert len(self.bag_times) == len(self.bag_x) == len(self.bag_y) == len(self.bag_z), "Plotter : self.bag_* aren't the same size after trimming"

        #rewrite bag_times to start at 0 and be written in [s] instead of [ns]
        bag_start_time = self.bag_times[0]
        self.bag_times =  (self.bag_times-bag_start_time) * (10**-9)
        assert self.bag_times[0] == 0
        print(f"trimmed bag_times starts: {self.bag_times[0]}s and ends: {self.bag_times[-1]}, size: {len(self.bag_times)}")



    def create_figures(self, ideal_csvfile:str, rosbag_csvfile:str, pdfname:str):
        '''Method that creates the pdf with the plots'''

        self.read_csv_and_set_arrays(ideal_csvfile,rosbag_csvfile)
        
        passed="failed"
        if self.test_passed():
            passed = "passed"
        
        #create PDF to save figures
        if(pdfname[-4:] != ".pdf"):
            pdfname= pdfname + '.pdf'

        #check if user wants to overwrite the report file
        self.file_guard(pdfname)
        pdf_pages = PdfPages(pdfname)

        #create title page
        if "figure8" in ideal_csvfile:
            name = "Figure8"
        elif "multi_trajectory" in ideal_csvfile:
            name = "Multi_trajectory"
        else:
            name = "Unnamed test"
            print("Plotter : test name not defined")

        text = f'{name} trajectory test'
        title_text_settings = f'Settings:\n'
        title_text_parameters = f'Parameters:\n'
        for key, value in self.params.items():
            title_text_parameters += f"    {key}: {value}\n"
        title_text_results = f'Results: test {passed}\n' + f'max error : '

        title_text = text + "\n" + title_text_settings + "\n" + title_text_parameters + "\n" + title_text_results
        fig = plt.figure(figsize=(5,8))
        fig.text(0.1, 0.1, title_text, size=11)
    

        pdf_pages.savefig(fig)
    
   
        #create plots
        fig, ax = plt.subplots()
        ax.plot(self.bag_times, self.ideal_traj_x, label='Ideal trajectory', linestyle="--", linewidth=1, zorder=10)
        ax.plot(self.bag_times, self.bag_x, label='Recorded trajectory')
        ax.set_xlabel('time [s]')
        ax.set_ylabel('x position [m]')  
        ax.set_title("Trajectory x")
        
        ax.grid(which='major', color='#DDDDDD', linewidth=0.8)
        ax.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
        ax.minorticks_on()
        fig.tight_layout(pad = 4)
        fig.legend()
        
        pdf_pages.savefig(fig) 
          
        fig2, ax2 = plt.subplots()
        ax2.plot(self.bag_times, self.ideal_traj_y, label='Ideal trajectory', linestyle="--", linewidth=1, zorder=10)
        ax2.plot(self.bag_times, self.bag_y, label='Recorded trajectory')
        ax2.set_xlabel('time [s]')
        ax2.set_ylabel('y position [m]')  
        ax2.set_title("Trajectory y")
        ax2.grid(which='major', color='#DDDDDD', linewidth=0.8)
        ax2.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
        ax2.minorticks_on()
        fig2.tight_layout(pad = 4) 
        fig2.legend()
        pdf_pages.savefig(fig2) 
          

        fig3, ax3 = plt.subplots()
        ax3.plot(self.bag_times, self.ideal_traj_z, label='Ideal trajectory', linestyle="--", linewidth=1, zorder=10)
        ax3.plot(self.bag_times, self.bag_z, label='Recorded trajectory')
        ax3.set_xlabel('time [s]')
        ax3.set_ylabel('z position [m]')   
        ax3.set_title("Trajectory z")
        ax3.grid(which='major', color='#DDDDDD', linewidth=0.8)
        ax3.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
        ax3.minorticks_on()
        fig3.tight_layout(pad = 4)
        fig3.legend()
        pdf_pages.savefig(fig3) 
 
        fig4, ax4 = plt.subplots()
        ax4.plot(self.bag_times, self.euclidian_dist)
        ax4.set_xlabel('time [s]')
        ax4.set_ylabel('Euclidean distance [m]')
        ax4.set_title('Deviation between ideal and recorded trajectories')
        fig4.tight_layout(pad=4)
        ax4.grid(which='major', color='#DDDDDD', linewidth=0.8)
        ax4.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
        ax4.minorticks_on()
        pdf_pages.savefig(fig4)


        fig5,ax5 = plt.subplots()
        ax5.plot(self.ideal_traj_x, self.ideal_traj_y, label='Ideal trajectory', linestyle="--", linewidth=1, zorder=10)
        ax5.plot(self.bag_x, self.bag_y, label='Recorded trajectory')
        ax5.set_xlabel('x [m]')
        ax5.set_ylabel('y [m]')
        ax5.set_title('2D visualization')
        fig5.tight_layout(pad=4)
        ax5.grid(which='major', color='#DDDDDD', linewidth=0.8)
        ax5.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
        ax5.minorticks_on()
        pdf_pages.savefig(fig5)


        fig6 = plt.figure()
        ax6 = fig6.add_subplot(projection="3d")
        ax6.plot3D(self.ideal_traj_x,self.ideal_traj_y,self.ALTITUDE_CONST_FIG8, label='Ideal trajectory', linestyle="--", linewidth=1, zorder=10)
        ax6.plot3D(self.bag_x,self.bag_y,self.bag_z, label='Recorded trajectory', linewidth=1)
        ax6.grid(True)
        ax6.set_title('3D visualization')
        ax6.set_xlabel('x [m]')
        ax6.set_ylabel('y [m]')
        ax6.set_zlabel('z [m]')
        plt.close(fig6)
        plt.tight_layout(pad=4)
        pdf_pages.savefig(fig6)



        pdf_pages.close()

        print("Results saved in " + pdfname)

    def test_passed(self) -> bool :
        '''Returns True if the deviation between recorded and ideal trajectories of the drone always stayed lower 
        than EPSILON. Returns False otherwise '''

        nb_dev_points = len(self.deviation)

        if nb_dev_points == 0:
            print("Test passed")
            return True
        else:
            print(f"The deviation between ideal and recorded trajectories is greater than {self.EPSILON}m for {nb_dev_points} "
                  f"datapoints, which corresponds to a duration of {nb_dev_points*0.01}s")
            return False

if __name__=="__main__":
    
    #command line utility 

    from argparse import ArgumentParser, Namespace
    parser = ArgumentParser(description="Creates a pdf plotting the recorded trajectory of a drone against its desired trajectory")
    parser.add_argument("desired_trajectory", type=str, help=".csv file containing (time,x,y,z) of the ideal/desired drone trajectory")
    parser.add_argument("recorded_trajectory", type=str, help=".csv file containing (time,x,y,z) of the recorded drone trajectory")
    parser.add_argument("pdf", type=str, help="name of the pdf file you want to create/overwrite")
    parser.add_argument("--open", help="Open the pdf directly after it is created", action="store_true")
    args : Namespace = parser.parse_args()

    plotter = Plotter()
    plotter.create_figures(args.desired_trajectory, args.recorded_trajectory, args.pdf)
    if args.open:
        import subprocess
        subprocess.call(["xdg-open", args.pdf])
