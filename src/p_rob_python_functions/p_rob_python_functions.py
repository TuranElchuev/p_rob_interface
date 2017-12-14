#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, json
from myp_ros.srv import Generic_myP_Service
    
cntrlFuncSrv = rospy.ServiceProxy("arm/connect", Generic_myP_Service)
scriptFncSrv = rospy.ServiceProxy("arm/run_script", Generic_myP_Service)

def callControlFunc(args):
    rospy.wait_for_service('arm/connect')
    try:
        resp = cntrlFuncSrv(args)
        return resp.output
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return None
        
def callScriptFunc(args):
    if init_app() is None:
        return None    
    
    rospy.wait_for_service('arm/run_script')
    try:
        resp = scriptFncSrv(args)
        fin_app()
        return resp.output
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        fin_app()
        return None

def makeArgs(func, args = ""):
    return "{\"func\": \"" + func +"\", \"args\": [" + args + "]}"
    
#Control functions
def recover():
    return callControlFunc(makeArgs("recover")) is not None

def connect():
    return callControlFunc(makeArgs("connect")) is not None
        
def calibrate():
    return callControlFunc(makeArgs("calibrate")) is not None
    
def init_app():
    return callControlFunc(makeArgs("initialize_application")) is not None
    
def fin_app():
    return callControlFunc(makeArgs("finalize_application")) is not None
    
def disconnect():
    return callControlFunc(makeArgs("connect")) is not None
        
def get_status():
    return callControlFunc(makeArgs("get_status"))
    
def init_prob():
    status = get_status()
    if status is None:
        return False
    elif status == "\"none\"":
        return (connect() is not None and calibrate() is not None)
    else:
        return True

#Script functions
def digitalOutput(pin, value):
    return callScriptFunc(makeArgs("write_digital_outputs", json.dumps({str(pin): bool(value)}))) is not None
  
def home():
    return callScriptFunc(makeArgs("move_joint", "[1, 2, 3, 4, 5, 6], [0, 0, 0, 0, 0, 0]")) is not None
