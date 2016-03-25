from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import division
import collections
import math
import time

import openravepy
import numpy
import yaml

import prpy.planning.base
import prpy.serialization


def serialize_value(val):
    if isinstance(val, openravepy.Robot):
        return val.GetName()
    elif isinstance(val, float):
        return float(val)
    elif isinstance(val, bool):
        return bool(val)
    elif isinstance(val, basestring):
        return str(val)
    elif isinstance(val, numpy.ndarray):
        if len(val.shape) == 1:
            return list(map(float,val))
        elif len(val.shape) == 2:
            matrix = []
            for i in range(val.shape[0]):
                matrix.append(list(map(float,val[i,:])))
            return matrix
        else:
            return str(val)
    elif isinstance(val, collections.Sequence):
        seq = []
        for i in val:
            seq.append(serialize_value(i))
        return seq
    else:
        return str(val)


class LoggedPlanner(prpy.planning.base.MetaPlanner):
    def __init__(self, planner):
        super(Logged, self).__init__()
        self.planner = planner
    
    def __str__(self):
        return 'Logged({0:s})'.format(self.planner)
    
    def has_planning_method(self, method_name):
        return self.planner.has_planning_method(method_name)
    
    def get_planning_method_names(self):
        return self.planner.get_planning_method_names()
    
    def get_planners(self, method_name):
        return [self.planner]
    
    def plan(self, method, args, kw_args):
        
        # get log file name
        stamp = time.time()
        struct = time.localtime(stamp)
        fn = 'log-{}.{:06d}'.format(
            time.strftime('%Y%m%d-%H%M%S', struct),
            int(1.0e6*(stamp-math.floor(stamp))))
        
        # serialize planning request
        reqdict = {}
        reqdict['method'] = method # string
        reqdict['args'] = []
        for arg in args:
            reqdict['args'].append(serialize_value(arg))
        reqdict['kw_args'] = {str(k):serialize_value(v) for k,v in kw_args.items()}
        
        # serialize environment
        # note, first args element assumed to be a robot
        envdict = {}
        envdict['kinbodies'] = {}
        for kb in args[0].GetEnv().GetBodies():
            name = kb.GetName()
            uri = kb.GetURI()
            serkb = prpy.serialization.serialize_kinbody(kb)
            if uri:
                del serkb['links']
                del serkb['joints']
                if kb.IsRobot():
                    del serkb['manipulators']
                del serkb['kinbody_state']['link_transforms']
            envdict['kinbodies'][name] = serkb
        
        plan_fn = getattr(self.planner, method)
        traj = plan_fn(*args, **kw_args)
        
        # serialize result
        resdict = {}
        resdict['traj_first'] = list(map(float,traj.GetWaypoint(0)))
        resdict['traj_last'] = list(map(float,traj.GetWaypoint(traj.GetNumWaypoints()-1)))
        
        # create yaml dictionary to be serialized
        yamldict = {}
        yamldict['environment'] = envdict
        yamldict['request'] = reqdict
        yamldict['result'] = resdict
        fp = open(fn,'w')
        yaml.safe_dump(yamldict, fp)
        fp.close()

        return traj
