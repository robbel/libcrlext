experiment_name = "sysadmin"

log_dir = "exp_log"

agent_echo = Agent("echo run an agent")
agent_random = Agent("random_agent");
agent_spudd = Agent("glue_spudd ./test/SPUDD-OPTDual-ma-sysadmin-ring-6.ADD", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext/libcrl")
agent_alp = Agent("glue_alp ring 6");

env_echo = Environment("echo run an environment")
glue_sysadmin1 = Environment("glue_sysadmin ring 6")



num_steps = 200
num_trials = 50
num_runs = 1

#instance names must be unique
instances = [
    #Instance("DEBUG", agent_echo, env_echo, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("RAND", agent_random, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("SPUDD", agent_spudd, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("ALP", agent_alp, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
]
