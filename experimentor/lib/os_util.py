import os, sys, pty

def launchProcess(hcmd, cwd, cmd, verbose, silent, **entries):
    id = os.fork()
    if id:
        return id
    prompt = '$'
    if cwd:
        prompt = cwd+prompt
        os.chdir(cwd)
    if verbose:
        if hcmd:
            print '@'+hcmd[0]+':', 
        print prompt , cmd, entries
    os.environ.update(entries)
    if not hcmd and silent:
        cmd += " > /dev/null"
    if not hcmd:
        os.system(cmd)
    else:
        envs = [item[0]+'='+item[1] for item in entries.items()]
        envs_str = reduce(lambda x,y:x+' '+y, envs)
        ssh_cmd = cmd
        if cwd:
            ssh_cmd = 'cd '+cwd+'\;'+cmd
        full_cmd = 'ssh '+hcmd[0]+' '+hcmd[2]+' \\"'+ssh_cmd+'\\" '+envs_str
        if silent:
            full_cmd += " > /dev/null"
        #print full_cmd
        os.system(full_cmd)
    os._exit(0)
    
def checkIfExists(dir, file):
    try:
        return file in os.listdir(dir)
    except:
        pass

def makeDirectory(dir):
    try:
        os.listdir(dir)
        return
    except:
        pass
    os.mkdir(dir)
