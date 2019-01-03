#! /usr/bin/python3
import json
from collections import OrderedDict
import pprint
import os.path

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
    # pprint.pprint(output_dict,width=40)
    # return json.dumps(output_dict)
    return output_dict

s = '{"coefficient_adc_raw_value_to_rad_per_sec"    :     0.123 ,"frequency"    :    60,"angular_velocity" : [[123,456,789,543,834,112],[984,564,464]]}'


pprint.pprint(rawToAngularVelocityJson(s), width=40)

with open(os.path.expanduser('~') + '/python_test/test2.json','w') as f:
    json.dump(rawToAngularVelocityJson(s), f, indent=4)