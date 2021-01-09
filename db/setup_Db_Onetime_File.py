# this script will be used to initialize assets for the robot

# set up the db collections
from tinydb import TinyDB
db = TinyDB('db/data/robotProperties.json')
db.insert({'property':'left_trim','value':0})
db.insert({'property':'right_trim','value':0})

print ('robot properties collection initialized')
