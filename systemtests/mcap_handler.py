from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
import rosbag2_py
import csv

class McapHandler:
    def __init__(self):
        pass

    def read_messages(self, input_bag: str):
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )
        topic_types = reader.get_all_topics_and_types()

        def typename(topic_name):
            for topic_type in topic_types:
                if topic_type.name == topic_name:
                    return topic_type.type
            raise ValueError(f"topic {topic_name} not in bag")

        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp
        del reader
    
    def write_mcap_to_csv(self, inputbag:str, outputfile:str):
        '''A method which translates an .mcap rosbag file format to a .csv file. 
        Only written to translate the /tf topic but could easily be extended to other topics'''

        try:
            print("Translating .mcap to .csv")
            f = open(outputfile, 'w+')
            writer = csv.writer(f)
            for topic, msg, timestamp in self.read_messages(inputbag):
                writer.writerow([timestamp, msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z])
            f.close()
        except FileNotFoundError:
            print(f"McapHandler : file {outputfile} not found")
            exit(1)




if __name__ == "__main__":

    #command line utility 

    from argparse import ArgumentParser, Namespace
    parser = ArgumentParser(description="Translates the /tf topic of an .mcap rosbag file format to a .csv file")
    parser.add_argument("inputbag", type=str, help="The .mcap rosbag file to be translated")
    parser.add_argument("outputfile", type=str, help="Output csv file that has to be created/overwritten")
    args:Namespace = parser.parse_args()

    translator =  McapHandler()
    translator.write_mcap_to_csv(args.inputbag,args.outputfile)
