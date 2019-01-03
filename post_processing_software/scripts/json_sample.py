#! /usr/bin/python3
import json
from collections import OrderedDict
import pprint

import os.path

dp = json.dumps(['foo',{'bar':('baz',None,1.0,2)}])
print(dp)
print(type(dp))

dp2 = json.dumps({'4':5,'6':7})
print(type(dp2))

data = json.loads(dp)
print(type(data))

s = r'{"C": "\u3042", "A": {"i": 1, "j": 2}, "B": [{"X": 1, "Y": 10}, {"X": 2, "Y": 20}]}'

print(s)
# {"C": "\u3042", "A": {"i": 1, "j": 2}, "B": [{"X": 1, "Y": 10}, {"X": 2, "Y": 20}]}

d = json.loads(s)

pprint.pprint(d, width=40)
# {'A': {'i': 1, 'j': 2},
#  'B': [{'X': 1, 'Y': 10},
#        {'X': 2, 'Y': 20}],
#  'C': 'あ'}

print(type(d))
# <class 'dict'>

print(d['B'])
print(d['B'][0]['X'])
with open(os.path.expanduser('~') + '/python_test/test.json','w') as f:
    json.dump(d, f, indent=4)


print(os.getcwd())