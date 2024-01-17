import math
import copy
thisdict = {
  "colors1": [1,2],
  "colors2": [3,4],
  "colors3": [5,6],
}
this = copy.deepcopy(thisdict)

this['colors1'].clear()
print(this)

print(thisdict)