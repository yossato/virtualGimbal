#! /usr/bin/python3
import json
from collections import OrderedDict
import pprint
import os.path
import datetime
from read_angular_velocity_from_virtualGimbal import readAngularVelocityFromVirtualGimbal
def rawToAngularVelocityJson(angular_velocity_json):
    input_dict = json.loads(angular_velocity_json, object_pairs_hook=OrderedDict)

    output_dict = OrderedDict()
    output_dict["camera_model_name"] = "ILCE-6500"
    output_dict["sd_card_quaternion"] = {
            "w" :   1.0,
            "x" :   0.0,
            "y" :   0.0,
            "z" :   0.0
        }
    output_dict["frequency"] = 60.0
    output_dict["angular_velocity_rad_per_sec"] = []
    
    coeff = input_dict['coefficient_adc_raw_value_to_rad_per_sec']
    output_dict['angular_velocity_rad_per_sec'] = list(map(lambda x: list(map(lambda y:y * coeff, x)) , input_dict['angular_velocity']))
    
    return output_dict

def saveJson(dict):
    filename = 'records/' + '{0:%Y-%m-%d_%H.%M.%S}'.format(datetime.datetime.now()) + '.json'
    with open( filename,'w') as f:
        json.dump(dict, f, indent=4)
    return filename

if __name__ == '__main__':
    s = rawToAngularVelocityJson(readAngularVelocityFromVirtualGimbal())
    filename = saveJson(s)

    #pprint.pprint(s, width=40)
    print('Finished: '+ filename)
    



