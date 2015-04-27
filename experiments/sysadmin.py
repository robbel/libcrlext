experiment_name = "sysadmin"

log_dir = "exp_log"

agent_echo = Agent("echo run an agent")
agent_random = Agent("random_agent");
agent_spudd = Agent("glue_spudd ./test/SPUDD-OPTDual-64bit-nonportable.ADD", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext/libcrl")

env_echo = Environment("echo run an environment")
glue_sysadmin1 = Environment("glue_sysadmin ring 4")



num_steps = 200
num_trials = 50
num_runs = 1

#instance names must be unique
instances = [
    #Instance("DEBUG", agent_echo, env_echo, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("RAND", agent_random, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("SPUDD", agent_spudd, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
]
