#!/usr/bin/python
import sys, os, time, math, thread, threading, random, getopt

if float(sys.version[:3]) <= 2.3:
    import sets
    global set
    set = sets.Set

from csv_util import *
from os_util import *

exp_quiet = True
env_quiet = True
agent_quiet = True


class Agent:
    def __init__(self, command, **entries):
        self.command = command
        self.cwd = None
        self.load = 1
        self.__dict__.update(entries)
class Environment:
    def __init__(self, command, **entries):
        self.command = command
        self.cwd = None
        self.__dict__.update(entries)
class Instance:
    def __init__(self, name, agent, env, **entries):
        self.steps = 0
        self.trials = 1
        self.runs = 1
        self.name = name
        self.agent = agent
        self.env = env
        self.cwd = None
        self.__dict__.update(entries)
    

def backupOldExperiments(log_dir, experiment_name):
    if not checkIfExists(log_dir, experiment_name):
        return
    exp_dir = log_dir+'/'+experiment_name
    makeDirectory(exp_dir+'/.archive')
    date = None
    try:
        date = open(exp_dir+'/.date').read()
    except:
        return
    
    archive_dir = exp_dir+'/.archive/'+date
    makeDirectory(archive_dir)
    
    files = os.listdir(exp_dir)
    for file in files:
        if file == '.archive':
            continue
        src = exp_dir+'/'+file
        dst = archive_dir+'/'+file
        os.system('mv "'+src+'" "'+dst+'"')
    

def setupInstanceDir(exp_dir, instance):
    instance_dir = exp_dir+'/'+instance.name
    makeDirectory(instance_dir)
    file = open(instance_dir+'/launch.txt', 'w')
    file.write("This instance launched with\n")
    file.write("Agent CWD = "+`instance.agent.cwd`+"\n")
    file.write("Agent CMD = "+`instance.agent.command`+"\n")
    file.write("Environment CWD = "+`instance.env.cwd`+"\n")
    file.write("Environment CMD= "+`instance.env.command`+"\n")
    file.write("Steps = "+`instance.steps`+"\n")
    file.write("Trials = "+`instance.trials`+"\n")
    file.write("Runs = "+`instance.runs`+"\n")
    file.close()
    return instance_dir

def performInstanceRun(host, (instance_dir, run, instance), rand_seed, experiment_name, port):
    run_str = `run`
    run_max = instance.runs
    digits = int(math.log(run_max, 10))+1
    run_str = (("0"*digits)+run_str)[-digits:]
    
    agent = instance.agent
    env = instance.env
    
    run_dir = instance_dir+'/'+run_str
    makeDirectory(run_dir)
    
    log_file = run_dir+'/results.csv'
    if checkIfExists(run_dir, 'results.csv'):
        print run_dir, "exists: skipping"
        return
    
    file = open(run_dir+'/launch.txt', 'w')
    file.write("This instance run launched with\n")
    file.write("Agent CWD = "+`instance.agent.cwd`+"\n")
    file.write("Agent CMD = "+`instance.agent.command`+"\n")
    file.write("Environment CWD = "+`instance.env.cwd`+"\n")
    file.write("Environment CMD= "+`instance.env.command`+"\n")
    file.write("Steps = "+`instance.steps`+"\n")
    file.write("Trials = "+`instance.trials`+"\n")
    file.write("Run = "+`run`+"\n")
    file.write("Seed = "+`rand_seed`+"\n")
    file.close()
    
    exp_host = 'localhost'
    if host:
        exp_host = host[0]
    
    serv_id = launchProcess(host, None, 'rl_glue', False, True, RLGLUE_PORT=`port`)
    time.sleep(.1) #so the server is always ready for the things connecting
    exp_str = 'glue_exp -seed '+`rand_seed`+' -csv '+`instance.trials`+' '+`instance.steps`+' "'+log_file+'" "'+experiment_name+'"'
    exp_id = launchProcess(None, None, exp_str, False, exp_quiet, RLGLUE_HOST=exp_host, RLGLUE_PORT=`port`)
    agent_id = launchProcess(host, agent.cwd, agent.command, False, agent_quiet, RLGLUE_HOST=exp_host, RLGLUE_PORT=`port`)
    env_id = launchProcess(host, env.cwd, env.command, False, env_quiet, RLGLUE_HOST=exp_host, RLGLUE_PORT=`port`)
    
    agent_str = agent.command
    if agent.cwd:
        agent_str = agent.cwd+'$ '+agent_str
    env_str = env.command
    if env.cwd:
        env_str = env.cwd+'$ '+env_str
    
    
    desc = 'On '+exp_host+':'+`port`+'\n'+instance_dir+'\nInstance run:'+`run`+'\n'+agent_str+'\n'+env_str+'\n'+`instance.trials`+' trials\n'
    print desc
    
    os.waitpid(exp_id, 0)
    os.kill(serv_id, 1)
    os.kill(agent_id, 1)
    os.kill(env_id, 1)
    
class HostQueue:
    def __init__(self, hosts):
        self.hosts = hosts
        self.host_loads = [(self.getHostLoad(host),)+host for host in self.hosts]
        self.host_loads.sort()
    def getHostLoad(self, host):
        os.system('ssh '+host[0]+' uptime > .tmp_uptime')
        uptime = open('.tmp_uptime').read()
        load = uptime.split()[-3][:-1]
        print host[0]+':', load
        return host[1]-float(load) #remaining processing power
    def acquireHost(self, load):
        host = self.host_loads[-1]
        host = (host[0]-load,)+host[1:]
        self.host_loads[-1] = host
        self.host_loads.sort()
        return host[1:]
    def releaseHost(self, host, load):
        for i in range(len(self.host_loads)):
            if host[0] == self.host_loads[i]:
                host = (host[0]+load,)+host[1:]
                self.host_loads[i] = host
        self.host_loads.sort()

class ExperimentThread(threading.Thread):
    def __init__(self, instanceRuns, hqueue, lock, thread_id, rand_seed, experiment_name, port):
        threading.Thread.__init__(self)
        self.instanceRuns = instanceRuns
        self.hqueue = hqueue
        self.lock = lock
        self.thread_id = thread_id
        self.rand_seed = rand_seed
        self.experiment_name = experiment_name
        self.port = port
    def run(self):
        while True:
            self.lock.acquire()
            if len(self.instanceRuns) == 0:
                self.lock.release()
                return
            instanceRun = self.instanceRuns.pop(0)
            
            self.lock.release()
            seed = self.rand_seed + instanceRun[1]
            host = None
            
            agent_load = instanceRun[2].agent.load
            
            if self.hqueue:
                host = self.hqueue.acquireHost(agent_load)
            performInstanceRun(host, instanceRun, seed, self.experiment_name, self.port)
            if self.hqueue:
                self.hqueue.releaseHost(host, agent_load)

def spawnThread(instanceRuns, hqueue, lock, thread_id, rand_seed, experiment_name, low_port):
    port = low_port+thread_id
    t = ExperimentThread(instanceRuns, hqueue, lock, thread_id, rand_seed, experiment_name, port)
    t.start()
    return t
    
def averageResults(instanceDir):
    runDirs = [instanceDir+'/'+d for d in os.listdir(instanceDir)]
    resultCSVs = []
    for dir in runDirs:
        try:
            os.listdir(dir)
            res_file = dir+'/results.csv'
            csv = readCSV(res_file)
            resultCSVs.append(csv)
        except:
            pass
    if len(resultCSVs) != 0:
        avgCSV = averageCSVs(resultCSVs)
        writeCSV(avgCSV, instanceDir+'/results.csv')
    
def augmentResults(exp_dir, names):
    instanceDirs = [exp_dir+'/'+d for d in os.listdir(exp_dir)]
    resultCSVs = {}
    for name in names:
        resultCSVs[name] = []
    for dir in instanceDirs:
        if dir.endswith('.archive'):
            continue
        try:
            for name in names:
                res_file = dir+'/results'+name+'.csv'
                csv = readCSV(res_file)
                resultCSVs[name].append(csv)
        except:
            pass
    for name, csvs in resultCSVs.items():
        augmentedCSV = augmentCSVs(csvs)
        writeCSV(augmentedCSV, exp_dir+'/results'+name+'.csv')
        print exp_dir+'/results'+name+'.csv'

def separateResults(dir, title):
    if not checkIfExists(dir, 'results.csv'):
        return
    csv = readCSV(dir+'/results.csv')
    data_top = 0
    while csv[data_top][0]:
        data_top += 1
    data_top += 1
    data_csv = getCSVRows(csv, data_top, None)
    names = set([""])
    for i in range(len(data_csv[0])):
        name_data = getCSVColumns(data_csv, i, i+1)
        name = name_data[0][0]
        names.add(name)
        data = [[title]]+getCSVRows(name_data, 1, None)
        writeCSV(data, dir+'/results'+name+'.csv')
    return names

def printUsage():
    print "Usage:", sys.argv[0], " [options] <experiment>"
    print " Options:"
    print "  -s <n>: seed the experiment with n"
    print "  -x <n>: run n experiments concurrently on any system"
    print "  -h <host file>: use the entries in the host file to run agents remotely"
    print "  -p <port>: the start of the port range"
    print "  -r: resume current or interrupted experiment"
    print "  --vagent: do not surpress agent console"
    print "  --venv: do not surpress environment console"
    print "  --vexp: do not surpress experiment console"
    print "  --ignore-lock: ignore any existing lock file"
    

def runExperimentor():
    exp_file = None
    rand_seed = random.randint(0,100000)
    concurrent_experiments = 1
    hosts = None
    low_port = 4096
    resume_previous = False
    ignore_lock = False
    single_instance = None
    max_runs = 0
    
    try:
        long_opts = ['vagent', 'vexp', 'venv', 'ignore-lock', 'instance=', 'run-max=']
        optlist, args = getopt.gnu_getopt(sys.argv[1:], 's:x:h:p:r', long_opts)
                                          
        if len(args) != 1:
            raise `args`
        exp_file = args[0]
        for opt, val in optlist:
            if opt == '-h':
                try:
                    exec open(val).read()
                    if not hosts:
                        raise ""
                except:
                    print "Bad hosts file"
                    sys.exit(1)
            if opt == '-s':
                rand_seed = int(val)
            if opt == '-x':
                concurrent_experiments = int(val)
            if opt == '-p':
                low_port = int(val)
            if opt == '-r':
                resume_previous = True
            if opt == '--vagent':
                agent_quiet = False
            if opt == '--vexp':
                exp_quiet = False
            if opt == '--venv':
                env_quiet = False
            if opt == '--ignore-lock':
                ignore_lock = True
            if opt == '--instance':
                single_instance = val
            if opt == '--run-max':
                max_runs = int(val)
    except Exception, e:
        print e
        printUsage()
        sys.exit(1)

    exp_data = open(exp_file).read()
    
    
    #set defaults
    experiment_name = "experiment"
    log_dir = "~/exp_log"
    instances = []
    
    exec exp_data
    
    print "Running experiment:", experiment_name
    print "Random seed:", rand_seed
    print "Concurrent experiments:", concurrent_experiments
    print "Low port:", low_port
    
    hqueue = None
    if hosts:
        print "Using hosts", `[host[0] for host in hosts]`
        hqueue = HostQueue(hosts)
    print
    if len(instances) == 0:
        print "No experiments listed"
        sys.exit(1)
        
    makeDirectory(log_dir)
    
    exp_dir = log_dir+'/'+experiment_name
    
    if not ignore_lock and checkIfExists(exp_dir, 'LOCK'):
        print "Experiment already in progress! (or LOCK file needs to be removed)"
        sys.exit(1)
    
    if not resume_previous:
        print "Backing up previous experiment"
        print
        backupOldExperiments(log_dir, experiment_name)
    
    makeDirectory(exp_dir)
    print "creating", exp_dir+'/LOCK'
    open(exp_dir+'/LOCK', 'w').write(time.strftime("%Y-%m-%d@%H:%M:%S"))
    
    
    try:
        if not checkIfExists(exp_dir, '.date'):
            open(exp_dir+'/.date', 'w').write(time.strftime("%Y-%m-%d@%H:%M:%S"))
        else:
            open(exp_dir+'/.date', 'a').write(time.strftime("+%Y-%m-%d@%H:%M:%S"))
        
        instanceDirs = [setupInstanceDir(exp_dir, instance) for instance in instances]
    
        instanceDirs = [setupInstanceDir(exp_dir, instance) for instance in instances]
            
        instanceRuns = []
        for i in range(len(instances)):
            instance = instances[i]
            if single_instance and instance.name!=single_instance:
                continue
            if max_runs > 0 and max_runs < instance.runs:
                instance.runs = max_runs
            instanceDir = instanceDirs[i]
            for run in range(instance.runs):
                #("VI", agent_vi, env, 0, 100, 1)
                instanceRuns += [(instanceDir, run, instance)]
        
        lock = threading.Lock()
        
        threads = []
        for i in range(concurrent_experiments):
            threads.append(spawnThread(instanceRuns, hqueue, lock, i, rand_seed, experiment_name, low_port))
        [thread.join() for thread in threads]
        
        names = set()
        for i in range(len(instances)):
            instance = instances[i]
            instanceDir = instanceDirs[i]
            averageResults(instanceDir)
            names_instance = separateResults(instanceDir, instance.name)
            if names_instance:
                names = names.union(names_instance)
        if len(names):
            augmentResults(exp_dir, names)
    except Exception, e:
        print e
    except KeyboardInterrupt:
        pass
    
    print "removing", exp_dir+'/LOCK'
    os.remove(exp_dir+'/LOCK')
    
