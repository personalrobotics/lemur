
import collections
import math
import time

import openravepy
import numpy
import yaml

import prpy.planning.base
from prpy.serialization import serialize_kinbody


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


class Logged(prpy.planning.base.MetaPlanner):
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
        return self.planner.get_planners(method_name)
    
    def plan(self, method, args, kw_args):
        
        # get log file name
        stamp = time.time()
        struct = time.localtime(stamp)
        fn = 'log-{}.{:06d}'.format(
            time.strftime('%Y%m%d-%H%M%S', struct),
            int(1.0e6*(stamp-math.floor(stamp))))
        
        # get robot (assumed to be first member of args)
        robot = args[0]
        
        yamldict = {}
        
        yamldict['method'] = method
        yamldict['args'] = []
        for arg in args:
            yamldict['args'].append(serialize_value(arg))
        yamldict['kw_args'] = {str(k):serialize_value(v) for k,v in kw_args.items()}
        
        yamldict['bodies'] = {}
        
        for kb in robot.GetEnv().GetBodies():
            name = kb.GetName()
            uri = kb.GetURI()
            serkb = serialize_kinbody(kb)
            if uri:
                del serkb['links']
                del serkb['joints']
                if kb.IsRobot():
                    del serkb['manipulators']
                del serkb['kinbody_state']['link_transforms']
            yamldict['bodies'][name] = serkb
            
                
                
        
        fp = open(fn,'w')
        yaml.safe_dump(yamldict, fp)
        fp.close()
        
        plan_fn = getattr(self.planner, method)
        result = plan_fn(*args, **kw_args)
        
        return result
