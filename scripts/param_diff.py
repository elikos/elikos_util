from lxml import etree
import os
import sys
import csv
from itertools import ifilter

defalut_parameter_location = "https://raw.githubusercontent.com/mavlink/qgroundcontrol/master/src/FirmwarePlugin/PX4/PX4ParameterFactMetaData.xml"


if len(sys.argv) != 2 or not sys.argv[1].endswith(".params"):
    print "Usage : 'python param_diff.py QGroundControllParameter.params"
    sys.exit(2)

params_file_name = sys.argv[1]

###
# Functions
###
def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


###
# File IO
###
fp = open(params_file_name, 'rb')
out_file = open("changed_params.csv", 'wb')

root = None
try:
    root = etree.parse("PX4ParameterFactMetaData.xml")
except IOError:
    print "No default parameter file, fetching it."
    os.system("wget " + defalut_parameter_location)
    root = etree.parse("PX4ParameterFactMetaData.xml")


###
# Parsing
###
writer = csv.writer(out_file)
writer.writerow(["Param name","Value","Default value","Change type"])

for row in csv.reader(ifilter(lambda row: row[0]!='#', fp), delimiter='\t'):
    param_name = row[2]
    param_value = row[3]

    xpath = "//parameter[@name='" + param_name + "']"
    elems = root.xpath(xpath)
    change_type = None
    default = None
    if len(elems) is 0:
        change_type = "Custom parameter"
        default = "None"
    else:
        val_type = elems[0].get("type")
        if val_type == "FLOAT":
            value_set = float(param_value)
            default = float(elems[0].get("default"))
            if isclose(value_set, default, rel_tol=1e-07):
                continue
            else:
                change_type = "Changed"
        elif val_type == "INT32":
            value_set = int(param_value)
            default = int(elems[0].get("default"))
            if value_set == default:
                continue
            else:
                change_type = "Changed"
        else:
            print "value type not known : {0}".format(val_type)
    writer.writerow([param_name, param_value, default, change_type])

fp.close()
out_file.close()
